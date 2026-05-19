/**
 *******************************************************************************
 * @file  uart.c
 * @brief USART1/USART4 应用层：DMA 回调入队，任务出队处理（与 CAN 相同模式）
 *******************************************************************************
 */

#include "uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
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

/* Uart_Printf 共享静态缓冲 + 互斥锁，避免每次调用占用 300+ 字节栈 */
static SemaphoreHandle_t    s_printf_mutex;
static char                 s_printf_buf[160];

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

    s_printf_mutex = xSemaphoreCreateMutex();
    if (NULL == s_printf_mutex) {
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

/* 格式化输出到 USART1（调试/CAN 打印用）
 * 使用静态缓冲 + mutex，避免每次调用占用 300+ 字节任务栈。 */
void Uart_Printf(const char *fmt, ...)
{
    va_list ap;
    int     n;

    if (NULL == s_printf_mutex) {
        /* 初始化前的早期打印：回退到栈缓冲（调度器未启动时不能用 mutex） */
        char early_buf[80];
        va_start(ap, fmt);
        n = vsnprintf(early_buf, sizeof(early_buf), fmt, ap);
        va_end(ap);
        if (n > 0) {
            if (n >= (int)sizeof(early_buf)) {
                n = (int)sizeof(early_buf) - 1;
            }
            (void)drv_uart1_send((const uint8_t *)early_buf, (uint16_t)n);
        }
        return;
    }

    xSemaphoreTake(s_printf_mutex, portMAX_DELAY);

    va_start(ap, fmt);
    n = vsnprintf(s_printf_buf, sizeof(s_printf_buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n >= (int)sizeof(s_printf_buf)) {
            n = (int)sizeof(s_printf_buf) - 1;
        }
        (void)drv_uart1_send((const uint8_t *)s_printf_buf, (uint16_t)n);
    }

    xSemaphoreGive(s_printf_mutex);
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
