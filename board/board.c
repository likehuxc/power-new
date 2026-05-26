/**
 *******************************************************************************
 * @file  board.c
 * @brief 板级初始化：外设寄存器解锁/加锁，系统时钟配置（MPLL@200MHz）
 *******************************************************************************
 */

#include "board.h"

/* XTAL 引脚：PH0/PH1 */
#define BOARD_XTAL_PORT     (GPIO_PORT_H)
#define BOARD_XTAL_PIN      (GPIO_PIN_00 | GPIO_PIN_01)

void Board_PeriphUnlock(void)
{
    LL_PERIPH_WE(LL_PERIPH_SEL);
}

void Board_PeriphLock(void)
{
    LL_PERIPH_WP(LL_PERIPH_SEL);
}

/* 系统时钟：外部晶振 12MHz，MPLL 倍频至 200MHz（12/3*100/2） */
void Board_Init(void)
{
    stc_clock_xtal_init_t stcXtalInit;
    stc_clock_pll_init_t  stcMpllInit;

    GPIO_AnalogCmd(BOARD_XTAL_PORT, BOARD_XTAL_PIN, ENABLE);
    (void)CLK_XtalStructInit(&stcXtalInit);
    (void)CLK_PLLStructInit(&stcMpllInit);

    CLK_SetClockDiv(CLK_BUS_CLK_ALL,
                    (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 |
                     CLK_PCLK1_DIV2 | CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    stcXtalInit.u8Mode       = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv        = CLK_XTAL_DRV_LOW;
    stcXtalInit.u8State      = CLK_XTAL_ON;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);
    // 1. 设置 PLL 参数 12MHz -> 200MHz (12/3*100/2)
    stcMpllInit.PLLCFGR              = 0UL;
    stcMpllInit.PLLCFGR_f.PLLM      = 3UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLN      = 100UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLP      = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLQ      = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLR      = 2UL - 1UL;
    stcMpllInit.u8PLLState           = CLK_PLL_ON;
    stcMpllInit.PLLCFGR_f.PLLSRC    = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcMpllInit);
    // 2. 等待 PLL 稳定
    while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL)) {
        ;
    }

    // 3. 设置 SRAM 等待周期
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    // 4. 设置 EFM 等待周期
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    // 5. 设置 GPIO 读等待周期
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    // 6. 设置 PWC 高速到高性能
    (void)PWC_HighSpeedToHighPerformance();
    // 7. 设置系统时钟源为 PLL
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
    // 8. 复位 EFM 缓存 RAM
    EFM_CacheRamReset(ENABLE);
    // 9. 禁用 EFM 缓存 RAM
    EFM_CacheRamReset(DISABLE);
    // 10. 使能 EFM 缓存
    EFM_CacheCmd(ENABLE);
}
