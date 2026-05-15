/**
 *******************************************************************************
 * @file  led.c
 *******************************************************************************
 */

#include "led.h"

#include "board.h"
#include "drv_led.h"

#define LED_BLINK_PERIOD_MS             1000UL

void Led_Init(void)
{
    DrvLed_Init();
}

void Led_Task(void)
{
    static uint32_t u32LastTick;
    uint32_t        u32NowTick;

    u32NowTick = Board_GetTick();
    if ((u32NowTick - u32LastTick) >= LED_BLINK_PERIOD_MS) {
        u32LastTick = u32NowTick;
        DrvLed_Toggle();
    }
}
