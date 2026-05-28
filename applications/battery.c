/**
 *******************************************************************************
 * @file  battery.c
 * @brief 电池应用任务：周期查询两块电池
 *******************************************************************************
 */

#include "battery.h"

#include "FreeRTOS.h"
#include "task.h"

#include "battery_comm.h"

void Battery_Task(void *param)
{
    (void)param;

    for (;;) {
        // 每隔2.5秒查询一次版本 防止电池休眠
        BatteryComm_SendGetVersion(BAT1);
        BatteryComm_SendGetVersion(BAT2);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}
