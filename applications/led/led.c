/**
 *******************************************************************************
 * @file  led.c
 *******************************************************************************
 */

#include "led.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drv_led.h"

#define LED_BLINK_PERIOD_MS             1000UL

void Led_Init(void)
{
    DrvLed_Init();
}

void Led_Task(void *param)
{
    (void)param;

    Led_Init();

    for (;;) {
        DrvLed_Toggle();
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_PERIOD_MS));
    }
}
