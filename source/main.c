/**
 *******************************************************************************
 * @file  main.c
 * @brief 入口；外设与应用逻辑拆分至 board / drv / app，调度由 FreeRTOS 负责。
 *******************************************************************************
 */

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
#include "board.h"
#include "hc32_ll_utility.h"

/* SysTick 由 FreeRTOS 接管；在此维护 Board_GetTick() 用的毫秒计数 */
void vApplicationTickHook(void)
{
    SysTick_IncTick();
}

/* 任务栈溢出钩子（需在 FreeRTOSConfig.h 中开启检测） */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationIdleHook(void)
{
}

int32_t main(void)
{
    Board_PeriphUnlock();
    Board_Init();
    App_Init();
    App_StartTasks();

    /* 启动后不再执行裸循环 App_Process() */
    vTaskStartScheduler();

    for (;;) {
    }
}
