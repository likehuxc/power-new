/**
 *******************************************************************************
 * @file  drv_uart_dma.h
 *******************************************************************************
 */

#ifndef DRV_UART_DMA_H__
#define DRV_UART_DMA_H__

#include <stdbool.h>
#include <stdint.h>

#define DRV_UART_DMA_FRAME_LEN_MAX 500U
#define DRV_UART_DMA_TX_BUF_LEN_MAX 128U

int32_t DrvUartDma_Init(void);
int32_t DrvUartDma_Send(const uint8_t *pu8Data, uint16_t u16Len);
bool DrvUartDma_ReadFrame(uint8_t *pu8Buf, uint16_t u16BufSize, uint16_t *pu16Len);

#endif /* DRV_UART_DMA_H__ */
