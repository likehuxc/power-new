/**
 *******************************************************************************
 * @file  uart.h
 *******************************************************************************
 */

#ifndef UART_H__
#define UART_H__

#include <stdint.h>

int32_t Uart_Init(void);
void Uart1_Task(void);
void Uart4_Task(void);
void Uart_Printf(const char *fmt, ...);

#endif /* UART_H__ */
