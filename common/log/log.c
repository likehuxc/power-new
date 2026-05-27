/**
 *******************************************************************************
 * @file  log.c
 * @brief 日志模块：Log_Printf 写入 StreamBuffer，UartLog_Task 统一发送到 USART1
 *******************************************************************************
 */

#include "log.h"

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "uart_dma.h"
#include "hc32_ll.h"

#define UART_LOG_STREAM_SIZE    1024U
#define UART_LOG_LINE_LEN       160U
#define UART_LOG_TX_BUF_LEN     128U

static StreamBufferHandle_t s_uart_log_stream;

int32_t Log_Init(void)
{
    s_uart_log_stream = xStreamBufferCreate(UART_LOG_STREAM_SIZE, 1U);
    if (NULL == s_uart_log_stream) {
        return LL_ERR;
    }
    return LL_OK;
}

/* 格式化输出到日志 StreamBuffer，由 UartLog_Task 统一发送到 USART1。
 * 支持中断上下文和任务上下文调用。 */
void Log_Printf(const char *fmt, ...)
{
    char    buf[UART_LOG_LINE_LEN];
    va_list ap;
    int     n;

    if ((NULL == fmt) || (NULL == s_uart_log_stream)) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }
    if (n >= (int)sizeof(buf)) {
        n = (int)sizeof(buf) - 1;
    }

    if (__get_IPSR() != 0U) {
        /* 中断上下文 */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        (void)xStreamBufferSendFromISR(s_uart_log_stream,
                                       buf, (size_t)n,
                                       &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        /* 任务上下文，不等待（避免阻塞业务线程，满则丢弃） */
        (void)xStreamBufferSend(s_uart_log_stream,
                                buf, (size_t)n, 0U);
    }
}

/* 日志发送任务：唯一调用 drv_uart1_send() 发送日志的出口 */
void UartLog_Task(void *param)
{
    uint8_t tx_buf[UART_LOG_TX_BUF_LEN];
    size_t  len;

    (void)param;

    for (;;) {
        len = xStreamBufferReceive(s_uart_log_stream, tx_buf, sizeof(tx_buf), portMAX_DELAY);
        if (len > 0U) {
            (void)drv_uart1_send(tx_buf, (uint16_t)len);
        }
    }
}
