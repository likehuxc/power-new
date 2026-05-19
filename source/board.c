/**
 *******************************************************************************
 * @file  board.c
 * @brief 板级时钟初始化（SysTick 由 FreeRTOS 配置）
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
}
