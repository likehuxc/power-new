/**
 *******************************************************************************
 * @file  board.c
 * @brief 板级时钟与 SysTick；tick 递增在 main.c 的 vApplicationTickHook 中
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
    /* 1ms 节拍；实际中断处理由 FreeRTOS 的 SysTick_Handler 进入 */
    (void)SysTick_Init(1000U);
}

uint32_t Board_GetTick(void)
{
    return SysTick_GetTick();
}
