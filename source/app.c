/**
 *******************************************************************************
 * @file  app.c
 * @brief 应用层：外设初始化与 FreeRTOS 任务创建
 *******************************************************************************
 */

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "battery.h"
#include "can_port.h"
#include "hc32_ll.h"
#include "led.h"
#include "uart.h"

/* 外设与应用模块初始化（在启动调度器之前调用） */
void App_Init(void)
{
    /* 与 d5 一致，便于 FreeRTOS 中断优先级分组 */
    NVIC_SetPriorityGrouping(3U);

    /* 初始化外设 */
    Led_Init();
    Uart_Init();
    CanPort_Init();
}

/* 创建 LED / USART1 / USART4 / CAN / 电池测试 任务 */
void App_StartTasks(void)
{
    (void)xTaskCreate(Led_Task, "led", 128, NULL, configMAX_PRIORITIES - 10, NULL);
    (void)xTaskCreate(Uart1_Task, "u1", 512, NULL, configMAX_PRIORITIES - 9, NULL);
//    (void)xTaskCreate(Uart4_Task, "u4", 256, NULL, configMAX_PRIORITIES - 8, NULL);
    (void)xTaskCreate(CanPort_Task, "can", 512, NULL, configMAX_PRIORITIES - 7, NULL);
    (void)xTaskCreate(Battery_Task, "bat", 512, NULL, configMAX_PRIORITIES - 6, NULL);
}
