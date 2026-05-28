/**
 *******************************************************************************
 * @file  uart_port.h
 *******************************************************************************
 */

#ifndef UART_PORT_H__
#define UART_PORT_H__

#include <stdint.h>

int32_t Uart_Init(void);
void Uart1_Task(void *param);
void Uart4_Task(void *param);

#endif /* UART_PORT_H__ */
