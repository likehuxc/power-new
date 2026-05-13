/**
 *******************************************************************************
 * @file  board.c
 *******************************************************************************
 */

#include "board.h"

#include "ev_hc32f460_lqfp100_v2_bsp.h"

void Board_PeriphUnlock(void)
{
    LL_PERIPH_WE(LL_PERIPH_SEL);
}

void Board_PeriphLock(void)
{
    LL_PERIPH_WP(LL_PERIPH_SEL);
}

void Board_Init(void)
{
    BSP_CLK_Init();
    (void)SysTick_Init(1000U);
}

void SysTick_Handler(void)
{
    SysTick_IncTick();

    __DSB();
}

uint32_t Board_GetTick(void)
{
    return SysTick_GetTick();
}
