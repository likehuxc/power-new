/**
 *******************************************************************************
 * @file  board.h
 *******************************************************************************
 */

#ifndef BOARD_H__
#define BOARD_H__

#include <stdint.h>
#include "hc32_ll.h"

#define LL_PERIPH_SEL \
    (LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | \
     LL_PERIPH_EFM | LL_PERIPH_SRAM)

#define RK3588_CAN_ID   (0x04U)
#define ORIN_CAN_ID     (0x05U)

void Board_PeriphUnlock(void);
void Board_PeriphLock(void);
void Board_Init(void);

/**
 * @brief  初始化 DWT 周期计数器（Cortex-M4）
 * @note   调用后可通过 DWT->CYCCNT 获取 CPU 周期计数，用于精确计时
 */
void Board_DWT_Init(void);

/**
 * @brief  获取当前 DWT 周期计数值
 * @retval DWT->CYCCNT 的当前值
 */
uint32_t Board_DWT_GetCycle(void);

/**
 * @brief  计算两个 DWT 周期计数之间的时间差（微秒）
 * @param  start  起始周期计数
 * @param  end    结束周期计数
 * @retval 时间差，单位微秒
 */
uint32_t Board_DWT_GetElapsedUs(uint32_t start, uint32_t end);

#endif /* BOARD_H__ */
