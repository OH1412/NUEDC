#include "bt_uart_protocol.h"

#include <string.h>


static uint16_t next_index(uint16_t index)
{
    index++;
    if (index >= BT_UART_RING_CAPACITY) {
        index = 0u;
    }
    return index;
}


static bool is_header(uint8_t byte)
{
    return byte == BT_CAR_TO_PC_HEADER || byte == BT_PC_TO_CAR_HEADER;
}


static uint8_t expected_footer(uint8_t header)
{
    return header == BT_CAR_TO_PC_HEADER
        ? BT_CAR_TO_PC_FOOTER
        : BT_PC_TO_CAR_FOOTER;
}


void bt_uart_ring_init(bt_uart_ring_t *ring)
{
    memset(ring, 0, sizeof(*ring));
}


bool bt_uart_ring_push_isr(bt_uart_ring_t *ring, uint8_t byte)
{
    const uint16_t next = next_index(ring->head);
    if (next == ring->tail) {
        ring->overflow_count++;
        return false;
    }
    ring->data[ring->head] = byte;
    ring->head = next;
    return true;
}


size_t bt_uart_ring_write_isr(
    bt_uart_ring_t *ring,
    const uint8_t *data,
    size_t length
)
{
    size_t accepted = 0u;
    while (accepted < length) {
        if (!bt_uart_ring_push_isr(ring, data[accepted])) {
            break;
        }
        accepted++;
    }
    return accepted;
}


bool bt_uart_ring_pop(bt_uart_ring_t *ring, uint8_t *byte)
{
    if (ring->tail == ring->head) {
        return false;
    }
    *byte = ring->data[ring->tail];
    ring->tail = next_index(ring->tail);
    return true;
}


void bt_uart_parser_init(bt_uart_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
    parser->state = BT_PARSER_WAIT_HEADER;
}


bool bt_uart_frame_is_valid(
    const uint8_t frame[BT_UART_FRAME_LENGTH]
)
{
    if (!is_header(frame[0])) {
        return false;
    }
    return frame[BT_UART_FRAME_LENGTH - 1u] == expected_footer(frame[0]);
}


size_t bt_uart_parser_process(
    bt_uart_parser_t *parser,
    bt_uart_ring_t *ring,
    bt_uart_frame_callback_t callback,
    void *user
)
{
    uint8_t byte = 0u;
    size_t emitted = 0u;

    while (bt_uart_ring_pop(ring, &byte)) {
        if (parser->state == BT_PARSER_WAIT_HEADER) {
            if (!is_header(byte)) {
                parser->discarded_bytes++;
                continue;
            }
            parser->frame[0] = byte;
            parser->index = 1u;
            parser->state = BT_PARSER_COLLECT_FRAME;
            continue;
        }

        parser->frame[parser->index++] = byte;
        if (parser->index < BT_UART_FRAME_LENGTH) {
            continue;
        }

        if (bt_uart_frame_is_valid(parser->frame)) {
            parser->valid_frames++;
            emitted++;
            if (callback != NULL) {
                callback(parser->frame, user);
            }
            parser->state = BT_PARSER_WAIT_HEADER;
            parser->index = 0u;
        } else {
            parser->invalid_frames++;
            /*
             * The last byte may itself be the next frame header. Keeping it
             * allows recovery from a lost byte without discarding that frame.
             */
            if (is_header(byte)) {
                parser->frame[0] = byte;
                parser->index = 1u;
                parser->state = BT_PARSER_COLLECT_FRAME;
            } else {
                parser->state = BT_PARSER_WAIT_HEADER;
                parser->index = 0u;
            }
        }
    }
    return emitted;
}
