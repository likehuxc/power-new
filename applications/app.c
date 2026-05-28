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
#include "log.h"
#include "uart_port.h"
#include "eeprom.h"
#include "bmi088.h"
#include "power_manage.h"

/* 外设与应用模块初始化（在启动调度器之前调用） */
void App_Init(void)
{
    /* 与 d5 一致，便于 FreeRTOS 中断优先级分组 */
    NVIC_SetPriorityGrouping(3U);

    /* 基础设施：串口和日志，所有 task 都可能用到，必须在调度器前就绪 */
    Uart_Init();
    Log_Init();
    PowerManage_Init();
}

/* 创建 LED / USART1 / USART4 / CAN / 电池测试 / 日志 任务 */
void App_StartTasks(void)
{
    (void)xTaskCreate(Led_Task, "led", 128, NULL, configMAX_PRIORITIES - 10, NULL);
    (void)xTaskCreate(Uart1_Task, "u1", 512, NULL, configMAX_PRIORITIES - 9, NULL);
//    (void)xTaskCreate(Uart4_Task, "u4", 256, NULL, configMAX_PRIORITIES - 8, NULL);
    (void)xTaskCreate(UartLog_Task, "uart_log", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    (void)xTaskCreate(CanPort_Task, "can", 512, NULL, configMAX_PRIORITIES - 7, NULL);
    (void)xTaskCreate(Battery_Task, "bat", 512, NULL, configMAX_PRIORITIES - 6, NULL);
    (void)xTaskCreate(Eeprom_Task, "eeprom", 512, NULL, configMAX_PRIORITIES - 5, NULL);
    (void)xTaskCreate(BMI088_Task, "bmi088", 512, NULL, configMAX_PRIORITIES - 4, NULL);
    (void)xTaskCreate(PowerManage_Task, "power_manage", 512, NULL, configMAX_PRIORITIES - 3, NULL);
}
