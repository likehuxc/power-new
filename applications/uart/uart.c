/**
 *******************************************************************************
 * @file  uart.c
 * @brief USART1/USART4 应用层：DMA 回调入队，任务出队处理（与 CAN 相同模式）
 *******************************************************************************
 */

#include "uart.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "drv_uart_dma.h"
#include "hc32_ll.h"

#define UART_BAUDRATE           115200UL
#define UART_RX_STREAM_SIZE     (16U * DRV_UART_DMA_FRAME_LEN_MAX)
#define UART_RX_TRIGGER_LEVEL   1U
#define UART_RX_TASK_BUF_LEN    DRV_UART_DMA_TX_BUF_LEN_MAX

static StreamBufferHandle_t s_uart1_rx_stream;
static StreamBufferHandle_t s_uart4_rx_stream;

/* USART1 接收回调：中断上下文，仅拷贝并入队 */
static void Uart1RecvCallback(const uint8_t *buf, uint16_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((NULL == buf) || (0U == len) || (NULL == s_uart1_rx_stream)) {
        return;
    }

    if (len > DRV_UART_DMA_FRAME_LEN_MAX) {
        len = DRV_UART_DMA_FRAME_LEN_MAX;
    }

    (void)xStreamBufferSendFromISR(s_uart1_rx_stream, buf, (size_t)len,
                                   &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* USART4 接收回调 */
static void Uart4RecvCallback(const uint8_t *buf, uint16_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((NULL == buf) || (0U == len) || (NULL == s_uart4_rx_stream)) {
        return;
    }

    if (len > DRV_UART_DMA_FRAME_LEN_MAX) {
        len = DRV_UART_DMA_FRAME_LEN_MAX;
    }

    (void)xStreamBufferSendFromISR(s_uart4_rx_stream, buf, (size_t)len,
                                   &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int32_t Uart_Init(void)
{
    int32_t ret;

    s_uart1_rx_stream = xStreamBufferCreate(UART_RX_STREAM_SIZE, UART_RX_TRIGGER_LEVEL);
    // s_uart4_rx_stream = xStreamBufferCreate(UART_RX_STREAM_SIZE, UART_RX_TRIGGER_LEVEL);
    if (NULL == s_uart1_rx_stream) {
        return LL_ERR;
    }

    ret = drv_uart1_init(UART_BAUDRATE);
    if (LL_OK != ret) {
        return ret;
    }
    drv_uart1_set_recv_callback(Uart1RecvCallback);
    /* 暂时关闭 USART4 */
    // ret = drv_uart4_init(UART_BAUDRATE);
    // if (LL_OK != ret) {
    //     return ret;
    // }
    // drv_uart4_set_recv_callback(Uart4RecvCallback);
    return LL_OK;
}

/* USART1：阻塞读队列，回显 */
void Uart1_Task(void *param)
{
    uint8_t rx_buf[UART_RX_TASK_BUF_LEN];
    size_t  len;

    (void)param;

    for (;;) {
        len = xStreamBufferReceive(s_uart1_rx_stream, rx_buf, sizeof(rx_buf), portMAX_DELAY);
        if (0U != len) {
            (void)drv_uart1_send(rx_buf, (uint16_t)len);
        }
    }
}

/* USART4：阻塞读队列，回显 */
void Uart4_Task(void *param)
{
    uint8_t rx_buf[UART_RX_TASK_BUF_LEN];
    size_t  len;

    (void)param;

    for (;;) {
        len = xStreamBufferReceive(s_uart4_rx_stream, rx_buf, sizeof(rx_buf), portMAX_DELAY);
        if (0U != len) {
            (void)drv_uart4_send(rx_buf, (uint16_t)len);
        }
    }
}
