/**
 *******************************************************************************
 * @file  uart.c
 *******************************************************************************
 */

#include "uart.h"

#include "drv_uart_dma.h"
#include "hc32_ll.h"
#include "ring_buf.h"

#define UART_BAUDRATE                   115200UL

/* 环形缓冲区：存放 USART1 已接收完成的一帧或多帧数据 */
#define UART1_RING_BUF_SIZE             (DRV_UART_DMA_FRAME_LEN_MAX + 64U)

static uint8_t          s_uart1_ring_storage[UART1_RING_BUF_SIZE];
static stc_ring_buf_t   s_uart1_ring_buf;

/* 接收回调：仅把本帧数据写入 ring buf（帧结束由驱动置 s_uart1_rx_frame_done） */
static void Uart1RecvCallback(const uint8_t *buf, uint16_t len)
{
    if ((NULL == buf) || (0U == len)) {
        return;
    }

    (void)BUF_Write(&s_uart1_ring_buf, (uint8_t *)buf, len);
}

int32_t Uart_Init(void)
{
    int32_t ret;

    ret = drv_uart1_init(UART_BAUDRATE);
    if (LL_OK != ret) {
        return ret;
    }

    ret = BUF_Init(&s_uart1_ring_buf, s_uart1_ring_storage, UART1_RING_BUF_SIZE);
    if (LL_OK != ret) {
        return ret;
    }

    drv_uart1_set_recv_callback(Uart1RecvCallback);
		
	return 1;

//    return drv_uart4_init(UART_BAUDRATE);
}

void Uart_Task(void)
{
    uint8_t  au8Buf[DRV_UART_DMA_TX_BUF_LEN_MAX];
    uint32_t u32Len;

    if (!drv_uart1_is_rx_frame_done()) {
        return;
    }

    while (!BUF_Empty(&s_uart1_ring_buf)) {
        u32Len = BUF_Read(&s_uart1_ring_buf, au8Buf, DRV_UART_DMA_TX_BUF_LEN_MAX);
        if (0U == u32Len) {
            break;
        }
        (void)drv_uart1_send(au8Buf, (uint16_t)u32Len);
    }

    drv_uart1_clear_rx_frame_done();
}
