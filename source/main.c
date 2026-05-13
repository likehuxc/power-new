/**
 *******************************************************************************
 * @file  usart/usart_uart_dma/source/main.c
 * @brief This example demonstrates UART data receive and transfer by DMA.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2022-10-31       CDT             Delete the redundant code
                                    Read USART_DR.RDR when USART overrun error occur.
   2023-01-15       CDT             Update UART timeout function calculating formula for Timer0 CMP value
   2023-09-30       CDT             Split register USART_DR to USART_RDR and USART_TDR
   2024-11-08       CDT             Optimize function: USART_TxComplete_IrqCallback
                                    Add function: USART_StopTimeoutTimer
   2026-05-13       User            Adapt to power-new: USART1 PA0(RX)/PA2(TX), PB7 heartbeat LED
                                    TX via polling (USART_UART_Trans), RX via DMA+TMR0 timeout
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"

/**
 * @addtogroup HC32F460_DDL_Examples
 * @{
 */

/**
 * @addtogroup USART_UART_DMA
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Peripheral register WE/WP selection */
#define LL_PERIPH_SEL                   (LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | \
                                         LL_PERIPH_EFM | LL_PERIPH_SRAM)

/* RX DMA: DMA1 CH0, triggered by USART1_RI */
#define RX_DMA_UNIT                     (CM_DMA1)
#define RX_DMA_CH                       (DMA_CH0)
#define RX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define RX_DMA_TRIG_SEL                 (AOS_DMA1_0)
#define RX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART1_RI)
#define RX_DMA_RECONF_TRIG_SEL          (AOS_DMA_RC)
#define RX_DMA_RECONF_TRIG_EVT_SRC      (EVT_SRC_AOS_STRG)
#define RX_DMA_TC_INT                   (DMA_INT_TC_CH0)
#define RX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH0)
#define RX_DMA_TC_IRQn                  (INT000_IRQn)
#define RX_DMA_TC_INT_SRC               (INT_SRC_DMA1_TC0)

/* TX DMA: DMA2 CH0, triggered by USART1_TI (reserved for future DMA TX use) */
#define TX_DMA_UNIT                     (CM_DMA2)
#define TX_DMA_CH                       (DMA_CH0)
#define TX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define TX_DMA_TRIG_SEL                 (AOS_DMA2_0)
#define TX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART1_TI)
#define TX_DMA_TC_INT                   (DMA_INT_TC_CH0)
#define TX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH0)
#define TX_DMA_TC_IRQn                  (INT001_IRQn)
#define TX_DMA_TC_INT_SRC               (INT_SRC_DMA2_TC0)

/* Timer0 for USART RX timeout detection */
#define TMR0_UNIT                       (CM_TMR0_1)
#define TMR0_CH                         (TMR0_CH_A)
#define TMR0_FCG_ENABLE()               (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE))

/* USART RX/TX pin definition (DS: Func32=USART1_TX, Func33=USART1_RX) */
#define USART_RX_PORT                   (GPIO_PORT_A)   /* PA0: USART1_RX */
#define USART_RX_PIN                    (GPIO_PIN_00)
#define USART_RX_GPIO_FUNC              (GPIO_FUNC_33)

#define USART_TX_PORT                   (GPIO_PORT_A)   /* PA2: USART1_TX */
#define USART_TX_PIN                    (GPIO_PIN_02)
#define USART_TX_GPIO_FUNC              (GPIO_FUNC_32)

/* USART unit definition */
#define USART_UNIT                      (CM_USART1)
#define USART_FCG_ENABLE()              (FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE))

/* USART interrupt definition */
#define USART_TX_CPLT_IRQn              (INT002_IRQn)
#define USART_TX_CPLT_INT_SRC           (INT_SRC_USART1_TCI)

#define USART_RX_ERR_IRQn               (INT003_IRQn)
#define USART_RX_ERR_INT_SRC            (INT_SRC_USART1_EI)

#define USART_RX_TIMEOUT_IRQn           (INT004_IRQn)
#define USART_RX_TIMEOUT_INT_SRC        (INT_SRC_USART1_RTO)

/* Baud rate */
#define USART_BAUDRATE                  (115200UL)

/* RX timeout: 20 bit-times (enough margin at 115200 baud) */
#define USART_TIMEOUT_BITS              (2000U)

/* RX buffer size */
#define APP_FRAME_LEN_MAX               (500U)

/* Polling TX timeout */
#define USART_TX_TIMEOUT_MS             (1000UL)

/* Heartbeat LED: PB7 */
#define HEART_LED_PORT                  (GPIO_PORT_B)
#define HEART_LED_PIN                   (GPIO_PIN_07)
#define HEART_LED_TOGGLE()              (GPIO_TogglePins(HEART_LED_PORT, HEART_LED_PIN))
#define HEART_LED_BLINK_PERIOD_MS       (1000UL)

/* Message sent every heartbeat period */
#define HEARTBEAT_MSG                   "AAA\r\n"

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static __IO en_flag_status_t m_enRxFrameEnd;
static __IO en_flag_status_t m_enTxBusy;
static __IO uint16_t         m_u16RxLen;
static uint8_t               m_au8RxBuf[APP_FRAME_LEN_MAX];
static uint8_t               m_au8HeartbeatMsg[] = HEARTBEAT_MSG;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void SysTick_Handler(void)
{
    SysTick_IncTick();

    __DSB();
}

static void HeartLed_Init(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState = PIN_STAT_SET;
    stcGpioInit.u16PinDir   = PIN_DIR_OUT;
    (void)GPIO_Init(HEART_LED_PORT, HEART_LED_PIN, &stcGpioInit);
}

/**
 * @brief  Send a buffer via USART polling TX.
 *         Enables USART_TX before sending; TX complete IRQ will disable it.
 */
static int32_t Usart_DmaSend(const uint8_t *pu8Data, uint16_t u16Len)
{
    if ((NULL == pu8Data) || (0U == u16Len)) {
        return LL_ERR_INVD_PARAM;
    }

    USART_FuncCmd(USART_UNIT, USART_TX, ENABLE);
    return USART_UART_Trans(USART_UNIT, pu8Data, u16Len, USART_TX_TIMEOUT_MS);
}

/**
 * @brief  Toggle heartbeat LED and send "AAA\r\n" every 1 s.
 */
static void HeartLed_Process(void)
{
    static uint32_t u32LastTick;
    uint32_t u32NowTick;

    u32NowTick = SysTick_GetTick();
    if ((u32NowTick - u32LastTick) >= HEART_LED_BLINK_PERIOD_MS) {
        u32LastTick = u32NowTick;
        HEART_LED_TOGGLE();
        (void)Usart_DmaSend(m_au8HeartbeatMsg, (uint16_t)(ARRAY_SZ(m_au8HeartbeatMsg) - 1U));
    }
}

/**
 * @brief  RX DMA transfer complete IRQ callback (buffer full).
 */
static void RX_DMA_TC_IrqCallback(void)
{
    m_enRxFrameEnd = SET;
    m_u16RxLen     = APP_FRAME_LEN_MAX;

    USART_FuncCmd(USART_UNIT, USART_RX_TIMEOUT, DISABLE);

    DMA_ClearTransCompleteStatus(RX_DMA_UNIT, RX_DMA_TC_FLAG);
}

/**
 * @brief  TX DMA transfer complete IRQ callback.
 *         Enables USART TC interrupt so the TC callback can clean up.
 */
static void TX_DMA_TC_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, USART_INT_TX_CPLT, ENABLE);

    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);
}

/**
 * @brief  Initialize DMA channels for USART RX and TX.
 * @retval int32_t  LL_OK on success, LL_ERR_INVD_PARAM otherwise.
 */
static int32_t DMA_Config(void)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    stc_dma_llp_init_t stcDmaLlpInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    static stc_dma_llp_descriptor_t stcLlpDesc;

    /* Enable DMA1, DMA2, and AOS clocks */
    RX_DMA_FCG_ENABLE();
    TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* ---- RX DMA: DMA1 CH0, USART1_RDR -> m_au8RxBuf ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8RxBuf);
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr   = (uint32_t)m_au8RxBuf;
    stcDmaInit.u32SrcAddr    = (uint32_t)(&USART_UNIT->RDR);
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    i32Ret = DMA_Init(RX_DMA_UNIT, RX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* Configure LLP so RX DMA auto-reloads when triggered by AOS_SW_Trigger */
        (void)DMA_LlpStructInit(&stcDmaLlpInit);
        stcDmaLlpInit.u32State = DMA_LLP_ENABLE;
        stcDmaLlpInit.u32Mode  = DMA_LLP_WAIT;
        stcDmaLlpInit.u32Addr  = (uint32_t)&stcLlpDesc;
        (void)DMA_LlpInit(RX_DMA_UNIT, RX_DMA_CH, &stcDmaLlpInit);

        stcLlpDesc.SARx   = stcDmaInit.u32SrcAddr;
        stcLlpDesc.DARx   = stcDmaInit.u32DestAddr;
        stcLlpDesc.DTCTLx = (stcDmaInit.u32TransCount << DMA_DTCTL_CNT_POS) |
                            (stcDmaInit.u32BlockSize   << DMA_DTCTL_BLKSIZE_POS);
        stcLlpDesc.LLPx   = (uint32_t)&stcLlpDesc;
        stcLlpDesc.CHCTLx = stcDmaInit.u32SrcAddrInc | stcDmaInit.u32DestAddrInc |
                            stcDmaInit.u32DataWidth   | stcDmaInit.u32IntEn       |
                            stcDmaLlpInit.u32State    | stcDmaLlpInit.u32Mode;

        DMA_ReconfigLlpCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
        DMA_ReconfigCmd(RX_DMA_UNIT, ENABLE);
        AOS_SetTriggerEventSrc(RX_DMA_RECONF_TRIG_SEL, RX_DMA_RECONF_TRIG_EVT_SRC);

        stcIrqSignConfig.enIntSrc    = RX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn      = RX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &RX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(RX_DMA_TRIG_SEL, RX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(RX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(RX_DMA_UNIT, RX_DMA_TC_INT, ENABLE);
        (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
    }

    /* ---- TX DMA: DMA2 CH0, m_au8RxBuf -> USART1_TDR (reserved) ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8RxBuf);
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr   = (uint32_t)(&USART_UNIT->TDR);
    stcDmaInit.u32SrcAddr    = (uint32_t)m_au8RxBuf;
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(TX_DMA_UNIT, TX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        stcIrqSignConfig.enIntSrc    = TX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn      = TX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &TX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(TX_DMA_TRIG_SEL, TX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(TX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(TX_DMA_UNIT, TX_DMA_TC_INT, ENABLE);
        /* TX DMA channel enabled on-demand when echoing received data */
    }

    return i32Ret;
}

/**
 * @brief  Configure TMR0 for USART RX timeout detection.
 * @param  [in] u16TimeoutBits  Number of bit-times for timeout period.
 */
static void TMR0_Config(uint16_t u16TimeoutBits)
{
    uint16_t u16Div;
    uint16_t u16Delay;
    uint16_t u16CompareValue;
    stc_tmr0_init_t stcTmr0Init;

    TMR0_FCG_ENABLE();

    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_XTAL32;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;

    if (TMR0_CLK_DIV1 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 7U;
    } else if (TMR0_CLK_DIV2 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 5U;
    } else if ((TMR0_CLK_DIV4  == stcTmr0Init.u32ClockDiv) ||
               (TMR0_CLK_DIV8  == stcTmr0Init.u32ClockDiv) ||
               (TMR0_CLK_DIV16 == stcTmr0Init.u32ClockDiv)) {
        u16Delay = 3U;
    } else {
        u16Delay = 2U;
    }

    u16Div          = (uint16_t)1U << (stcTmr0Init.u32ClockDiv >> TMR0_BCONR_CKDIVA_POS);
    u16CompareValue = ((u16TimeoutBits + u16Div - 1U) / u16Div) - u16Delay;
    stcTmr0Init.u16CompareValue = u16CompareValue;
    (void)TMR0_Init(TMR0_UNIT, TMR0_CH, &stcTmr0Init);

    TMR0_HWStartCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
}

/**
 * @brief  Forcibly stop the TMR0 timeout timer.
 *         Required when the frame ends via DMA-full rather than timeout.
 */
static void USART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch)
{
    uint32_t u32ClrMask;
    uint32_t u32SetMask;
    uint32_t u32BitOffset;

    u32BitOffset = 16UL * u32Ch;

    /* Step 1: SYNCLKA=1, SYNSA=0 */
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA) << u32BitOffset;
    u32SetMask =  TMR0_BCONR_SYNCLKA                     << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);

    /* Step 2: CSTA=0, SYNCLKA=0, SYNSA=1 */
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA | TMR0_BCONR_CSTA) << u32BitOffset;
    u32SetMask =  TMR0_BCONR_SYNSA                                           << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);
}

/**
 * @brief  USART RX timeout IRQ callback.
 *         Signals end-of-frame and triggers DMA reconfig via AOS software trigger.
 */
static void USART_RxTimeout_IrqCallback(void)
{
    if (m_enRxFrameEnd != SET) {
        m_enRxFrameEnd = SET;
        m_u16RxLen = APP_FRAME_LEN_MAX - (uint16_t)DMA_GetTransCount(RX_DMA_UNIT, RX_DMA_CH);

        /* Reload RX DMA via LLP */
        AOS_SW_Trigger();
    }

    USART_StopTimeoutTimer(TMR0_UNIT, TMR0_CH);

    USART_ClearStatus(USART_UNIT, USART_FLAG_RX_TIMEOUT);
}

/**
 * @brief  USART TX complete IRQ callback.
 *         Disables TX after polling transmission finishes.
 */
static void USART_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    m_enTxBusy = RESET;
}

/**
 * @brief  USART RX error IRQ callback.
 */
static void USART_RxError_IrqCallback(void)
{
    (void)USART_ReadData(USART_UNIT);

    USART_ClearStatus(USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

/**
 * @brief  Main function of UART DMA project
 * @param  None
 * @retval int32_t return value, if needed
 */
int32_t main(void)
{
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;

    /* MCU Peripheral registers write unprotected */
    LL_PERIPH_WE(LL_PERIPH_SEL);

    /* Initialize BSP system clock */
    BSP_CLK_Init();

    /* SysTick at 1 ms for HeartLed timing */
    (void)SysTick_Init(1000U);

    /* Initialize heartbeat LED on PB7 */
    HeartLed_Init();

    /* Initialize DMA (RX DMA1/CH0 + TX DMA2/CH0) */
    (void)DMA_Config();

    /* Initialize TMR0 timeout for RX frame-end detection */
    TMR0_Config(USART_TIMEOUT_BITS);

    /* Configure USART RX/TX pins */
    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    /* Enable USART peripheral clock */
    USART_FCG_ENABLE();

    /* Initialize UART */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv      = USART_CLK_DIV64;
    stcUartInit.u32Baudrate      = USART_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(USART_UNIT, &stcUartInit, NULL)) {
        for (;;) {
            HeartLed_Process();
        }
    }

    /* Register USART TX complete IRQ */
    stcIrqSigninConfig.enIRQn      = USART_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc    = USART_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Register USART RX error IRQ */
    stcIrqSigninConfig.enIRQn      = USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc    = USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxError_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Register USART RX timeout IRQ */
    stcIrqSigninConfig.enIRQn      = USART_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc    = USART_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* MCU Peripheral registers write protected */
    LL_PERIPH_WP(LL_PERIPH_SEL);

    /* Enable RX, RX interrupt, RX timeout function */
    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT |
                               USART_INT_RX_TIMEOUT), ENABLE);

    for (;;) {
        HeartLed_Process();

        if (SET == m_enRxFrameEnd) {
            m_enRxFrameEnd = RESET;
            /* m_au8RxBuf[0..m_u16RxLen-1] contains the received frame */
        }
    }
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
