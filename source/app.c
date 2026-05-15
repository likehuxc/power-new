/**
 *******************************************************************************
 * @file  app.c
 *******************************************************************************
 */

#include "app.h"

#include "can.h"
#include "led.h"
#include "uart.h"

void App_Init(void)
{
    Led_Init();
    Uart_Init();
    Can_Init();
}

void App_Process(void)
{
    Led_Task();
    Uart_Task();
    Can_Task();
}
