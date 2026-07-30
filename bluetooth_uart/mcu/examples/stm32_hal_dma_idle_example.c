/*
 * Adapt huart2/hdma_usart2_rx and chassis_* functions to your STM32 project.
 * RX uses HAL ReceiveToIdle DMA; TX uses interrupt mode for each 8-byte frame.
 */

#include "main.h"
#include "bt_uart_protocol.h"

#include <stdbool.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

static uint8_t uart_dma_rx[64];
static bt_uart_ring_t uart_ring;
static bt_uart_parser_t uart_parser;
static volatile bool uart_tx_busy = false;

static const uint8_t car_complete[BT_UART_FRAME_LENGTH] = {
    0x76, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67
};
static const uint8_t pc_ack_forward[BT_UART_FRAME_LENGTH] = {
    0x92, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29
};
static const uint8_t pc_ack_turn[BT_UART_FRAME_LENGTH] = {
    0x92, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29
};

typedef enum {
    CAR_STATE_IDLE = 0,
    CAR_STATE_FORWARD_1M,
    CAR_STATE_WAIT_ACK_10,
    CAR_STATE_TURN_LEFT_90,
    CAR_STATE_WAIT_ACK_11,
    CAR_STATE_FORWARD_05M,
    CAR_STATE_FINISHED,
    CAR_STATE_ERROR
} car_state_t;

static car_state_t car_state = CAR_STATE_IDLE;

/* Replace these with encoder/IMU closed-loop chassis functions. */
extern void chassis_forward_metres(float distance_m);
extern void chassis_turn_left_degrees(float angle_deg);
extern bool chassis_motion_done(void);
extern void chassis_stop(void);


static bool bt_send_frame(const uint8_t frame[BT_UART_FRAME_LENGTH])
{
    if (uart_tx_busy) {
        return false;
    }
    uart_tx_busy = true;
    if (HAL_UART_Transmit_IT(
            &huart2,
            (uint8_t *)frame,
            BT_UART_FRAME_LENGTH
        ) != HAL_OK) {
        uart_tx_busy = false;
        return false;
    }
    return true;
}


static void on_bluetooth_frame(
    const uint8_t frame[BT_UART_FRAME_LENGTH],
    void *user
)
{
    (void)user;
    if (car_state == CAR_STATE_WAIT_ACK_10
        && memcmp(frame, pc_ack_forward, BT_UART_FRAME_LENGTH) == 0) {
        chassis_turn_left_degrees(90.0f);
        car_state = CAR_STATE_TURN_LEFT_90;
    } else if (car_state == CAR_STATE_WAIT_ACK_11
               && memcmp(frame, pc_ack_turn, BT_UART_FRAME_LENGTH) == 0) {
        chassis_forward_metres(0.5f);
        car_state = CAR_STATE_FORWARD_05M;
    }
}


void bluetooth_uart_start(void)
{
    bt_uart_ring_init(&uart_ring);
    bt_uart_parser_init(&uart_parser);
    HAL_UARTEx_ReceiveToIdle_DMA(
        &huart2,
        uart_dma_rx,
        sizeof(uart_dma_rx)
    );
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


void car_mission_start(void)
{
    chassis_forward_metres(1.0f);
    car_state = CAR_STATE_FORWARD_1M;
}


/* Call frequently from the non-interrupt main loop. */
void car_mission_tick(void)
{
    bt_uart_parser_process(
        &uart_parser,
        &uart_ring,
        on_bluetooth_frame,
        NULL
    );

    if (car_state == CAR_STATE_FORWARD_1M && chassis_motion_done()) {
        chassis_stop();
        if (bt_send_frame(car_complete)) {
            car_state = CAR_STATE_WAIT_ACK_10;
        } else {
            car_state = CAR_STATE_ERROR;
        }
    } else if (
        car_state == CAR_STATE_TURN_LEFT_90 && chassis_motion_done()
    ) {
        chassis_stop();
        if (bt_send_frame(car_complete)) {
            car_state = CAR_STATE_WAIT_ACK_11;
        } else {
            car_state = CAR_STATE_ERROR;
        }
    } else if (
        car_state == CAR_STATE_FORWARD_05M && chassis_motion_done()
    ) {
        chassis_stop();
        car_state = CAR_STATE_FINISHED;
    }
}


void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t size
)
{
    if (huart->Instance != huart2.Instance) {
        return;
    }
    bt_uart_ring_write_isr(&uart_ring, uart_dma_rx, size);
    HAL_UARTEx_ReceiveToIdle_DMA(
        &huart2,
        uart_dma_rx,
        sizeof(uart_dma_rx)
    );
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart2.Instance) {
        uart_tx_busy = false;
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart2.Instance) {
        HAL_UARTEx_ReceiveToIdle_DMA(
            &huart2,
            uart_dma_rx,
            sizeof(uart_dma_rx)
        );
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}
