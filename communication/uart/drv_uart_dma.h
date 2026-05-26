/**
 *******************************************************************************
 * @file  drv_uart_dma.h
 *******************************************************************************
 */

#ifndef DRV_UART_DMA_H__
#define DRV_UART_DMA_H__

#include <stdint.h>

#define DRV_UART_DMA_FRAME_LEN_MAX      256U
#define DRV_UART_DMA_TX_BUF_LEN_MAX     256U

typedef void (*drv_uart_recv_cb_t)(const uint8_t *buf, uint16_t len);

int drv_uart1_init(uint32_t baudrate);
int drv_uart1_send(const uint8_t *buf, uint16_t len);
void drv_uart1_set_recv_callback(drv_uart_recv_cb_t cb);

int drv_uart4_init(uint32_t baudrate);
int drv_uart4_send(const uint8_t *buf, uint16_t len);
void drv_uart4_set_recv_callback(drv_uart_recv_cb_t cb);

#endif /* DRV_UART_DMA_H__ */
