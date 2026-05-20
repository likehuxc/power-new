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

/* 任务栈溢出钩子（需在 FreeRTOSConfig.h 中开启检测） */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    taskDISABLE_INTERRUPTS();

    /* 直接用底层发送，不走 Log_Printf（此时栈已不可信） */
    {
        static const char prefix[] = "\r\n[FATAL] Stack overflow: ";
        static const char suffix[] = "\r\n";
        extern int drv_uart1_send(const uint8_t *buf, uint16_t len);
        uint16_t name_len = 0;

        drv_uart1_send((const uint8_t *)prefix, (uint16_t)(sizeof(prefix) - 1U));
        if (pcTaskName != NULL) {
            while (pcTaskName[name_len] != '\0' && name_len < 8U) {
                name_len++;
            }
            drv_uart1_send((const uint8_t *)pcTaskName, name_len);
        }
        drv_uart1_send((const uint8_t *)suffix, (uint16_t)(sizeof(suffix) - 1U));
    }

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
