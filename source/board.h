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

void Board_PeriphUnlock(void);
void Board_PeriphLock(void);
void Board_Init(void);
uint32_t Board_GetTick(void);

#endif /* BOARD_H__ */
