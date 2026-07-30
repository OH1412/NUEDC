#include "bt_uart_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t captured[8][BT_UART_FRAME_LENGTH];
static size_t captured_count = 0u;


static void capture(
    const uint8_t frame[BT_UART_FRAME_LENGTH],
    void *user
)
{
    (void)user;
    assert(captured_count < 8u);
    memcpy(captured[captured_count++], frame, BT_UART_FRAME_LENGTH);
}


int main(void)
{
    const uint8_t complete[8] = {
        0x76, 0x01, 0, 0, 0, 0, 0, 0x67
    };
    const uint8_t ack10[8] = {
        0x92, 0x10, 0, 0, 0, 0, 0, 0x29
    };
    const uint8_t ack11[8] = {
        0x92, 0x11, 0, 0, 0, 0, 0, 0x29
    };
    bt_uart_ring_t ring;
    bt_uart_parser_t parser;

    bt_uart_ring_init(&ring);
    bt_uart_parser_init(&parser);

    /* A frame split across two DMA callbacks. */
    assert(bt_uart_ring_write_isr(&ring, complete, 3u) == 3u);
    assert(bt_uart_parser_process(&parser, &ring, capture, NULL) == 0u);
    assert(bt_uart_ring_write_isr(&ring, complete + 3u, 5u) == 5u);
    assert(bt_uart_parser_process(&parser, &ring, capture, NULL) == 1u);

    /* Noise plus two sticky frames in one DMA callback. */
    {
        uint8_t block[18];
        block[0] = 0x00;
        block[1] = 0xff;
        memcpy(block + 2u, ack10, 8u);
        memcpy(block + 10u, ack11, 8u);
        assert(bt_uart_ring_write_isr(&ring, block, sizeof(block))
               == sizeof(block));
        assert(bt_uart_parser_process(&parser, &ring, capture, NULL) == 2u);
    }

    /* Invalid footer must not prevent the following valid frame. */
    {
        uint8_t bad[8];
        memcpy(bad, complete, sizeof(bad));
        bad[7] = 0x00;
        assert(bt_uart_ring_write_isr(&ring, bad, sizeof(bad))
               == sizeof(bad));
        assert(bt_uart_ring_write_isr(&ring, complete, sizeof(complete))
               == sizeof(complete));
        assert(bt_uart_parser_process(&parser, &ring, capture, NULL) == 1u);
    }

    assert(captured_count == 4u);
    assert(memcmp(captured[0], complete, 8u) == 0);
    assert(memcmp(captured[1], ack10, 8u) == 0);
    assert(memcmp(captured[2], ack11, 8u) == 0);
    assert(memcmp(captured[3], complete, 8u) == 0);
    assert(parser.valid_frames == 4u);
    assert(parser.invalid_frames == 1u);
    assert(parser.discarded_bytes == 2u);
    assert(ring.overflow_count == 0u);

    puts("bt_uart_protocol: all tests passed");
    return 0;
}
