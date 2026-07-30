#ifndef BT_UART_PROTOCOL_H
#define BT_UART_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BT_UART_RING_CAPACITY 256u
#define BT_UART_FRAME_LENGTH 8u

#define BT_CAR_TO_PC_HEADER 0x76u
#define BT_CAR_TO_PC_FOOTER 0x67u
#define BT_PC_TO_CAR_HEADER 0x92u
#define BT_PC_TO_CAR_FOOTER 0x29u

typedef struct {
    uint8_t data[BT_UART_RING_CAPACITY];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflow_count;
} bt_uart_ring_t;

typedef void (*bt_uart_frame_callback_t)(
    const uint8_t frame[BT_UART_FRAME_LENGTH],
    void *user
);

typedef enum {
    BT_PARSER_WAIT_HEADER = 0,
    BT_PARSER_COLLECT_FRAME
} bt_uart_parser_state_t;

typedef struct {
    bt_uart_parser_state_t state;
    uint8_t frame[BT_UART_FRAME_LENGTH];
    uint8_t index;
    uint32_t valid_frames;
    uint32_t invalid_frames;
    uint32_t discarded_bytes;
} bt_uart_parser_t;

void bt_uart_ring_init(bt_uart_ring_t *ring);

/* Call from one ISR/DMA callback only. Returns false on overflow. */
bool bt_uart_ring_push_isr(bt_uart_ring_t *ring, uint8_t byte);

/* Push an entire DMA block; returns the number of bytes accepted. */
size_t bt_uart_ring_write_isr(
    bt_uart_ring_t *ring,
    const uint8_t *data,
    size_t length
);

/* Call from the main loop only. */
bool bt_uart_ring_pop(bt_uart_ring_t *ring, uint8_t *byte);

void bt_uart_parser_init(bt_uart_parser_t *parser);

/* Drain all currently buffered bytes and emit every valid frame. */
size_t bt_uart_parser_process(
    bt_uart_parser_t *parser,
    bt_uart_ring_t *ring,
    bt_uart_frame_callback_t callback,
    void *user
);

bool bt_uart_frame_is_valid(
    const uint8_t frame[BT_UART_FRAME_LENGTH]
);

#ifdef __cplusplus
}
#endif

#endif
