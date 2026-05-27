/**
 *******************************************************************************
 * @file  uart1_dma.c
 * @brief USART1 DMA 驱动
 *******************************************************************************
 */

#include "uart_dma.h"

#include <stddef.h>
#include <string.h>

#include "board.h"
#include "hc32_ll.h"

/* 接收DMA：DMA1 通道0，触发源�?USART1_RI */
#define UART1_RX_DMA_UNIT               (CM_DMA1)
#define UART1_RX_DMA_CH                 (DMA_CH0)
#define UART1_RX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define UART1_RX_DMA_TRIG_SEL           (AOS_DMA1_0)
#define UART1_RX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART1_RI)
#define UART1_RX_DMA_TC_INT             (DMA_INT_TC_CH0)
#define UART1_RX_DMA_TC_FLAG            (DMA_FLAG_TC_CH0)
#define UART1_RX_DMA_TC_IRQn            (INT000_IRQn)
#define UART1_RX_DMA_TC_INT_SRC         (INT_SRC_DMA1_TC0)

/* 发送DMA：DMA2 通道0，触发源�?USART1_TI */
#define UART1_TX_DMA_UNIT               (CM_DMA2)
#define UART1_TX_DMA_CH                 (DMA_CH0)
#define UART1_TX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define UART1_TX_DMA_TRIG_SEL           (AOS_DMA2_0)
#define UART1_TX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART1_TI)
#define UART1_TX_DMA_TC_INT             (DMA_INT_TC_CH0)
#define UART1_TX_DMA_TC_FLAG            (DMA_FLAG_TC_CH0)
#define UART1_TX_DMA_TC_IRQn            (INT001_IRQn)
#define UART1_TX_DMA_TC_INT_SRC         (INT_SRC_DMA2_TC0)

/* TMR0：用于USART1接收超时检�?*/
#define UART1_TMR0_UNIT                 (CM_TMR0_1)
#define UART1_TMR0_CH                   (TMR0_CH_A)
#define UART1_TMR0_FCG_ENABLE()         (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE))

/* USART1 收发引脚定义（数据手册：Func32=USART1_TX，Func33=USART1_RX�?*/
#define UART1_RX_PORT                   (GPIO_PORT_A)   /* PA0：USART1_RX */
#define UART1_RX_PIN                    (GPIO_PIN_00)
#define UART1_RX_GPIO_FUNC              (GPIO_FUNC_33)

#define UART1_TX_PORT                   (GPIO_PORT_A)   /* PA2：USART1_TX */
#define UART1_TX_PIN                    (GPIO_PIN_02)
#define UART1_TX_GPIO_FUNC              (GPIO_FUNC_32)

/* USART1 外设单元 */
#define UART1_UNIT                      (CM_USART1)
#define UART1_FCG_ENABLE()              (FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE))

/* USART1 中断定义 */
#define UART1_TX_CPLT_IRQn              (INT002_IRQn)
#define UART1_TX_CPLT_INT_SRC           (INT_SRC_USART1_TCI)

#define UART1_RX_ERR_IRQn               (INT003_IRQn)
#define UART1_RX_ERR_INT_SRC            (INT_SRC_USART1_EI)

#define UART1_RX_TIMEOUT_IRQn           (INT004_IRQn)
#define UART1_RX_TIMEOUT_INT_SRC        (INT_SRC_USART1_RTO)

/* 接收超时位数（帧间隙超过此位时间则判定帧结束�?*/
#define UART1_TIMEOUT_BITS              (2000U)

static __IO en_flag_status_t s_uart1_rx_frame_end;        /* 接收帧结束标�?*/
static __IO en_flag_status_t s_uart1_tx_busy;            /* 发送忙标志 */
static __IO uint16_t         s_uart1_rx_len;            /* 本帧实际接收字节�?*/
static uint8_t               s_uart1_rx_buf[UART_DMA_FRAME_LEN_MAX] = {0};   /* 接收缓冲�?*/
static uint8_t               s_uart1_tx_buf[UART_DMA_TX_BUF_LEN_MAX] = {0};   /* 发送缓冲区 */
static drv_uart_recv_cb_t    s_uart1_recv_cb;            /* 接收回调函数 */

static void UART1_StopTimeoutTimer(void);
static uint16_t UART1_CalcTimeoutCompareValue(uint16_t timeout_bits, uint32_t clock_div);
static void UART1_RestartRxDma(void);
static void UART1_NotifyRecv(const uint8_t *buf, uint16_t len);
static int32_t UART1_DMA_Config(void);
static void UART1_TMR0_Config(uint16_t timeout_bits);
static void UART1_RX_DMA_TC_IrqCallback(void);
static void UART1_TX_DMA_TC_IrqCallback(void);
static void UART1_RxTimeout_IrqCallback(void);
static void UART1_TxComplete_IrqCallback(void);
static void UART1_RxError_IrqCallback(void);

static void UART1_NotifyRecv(const uint8_t *buf, uint16_t len)
{
    if ((NULL != s_uart1_recv_cb) && (NULL != buf) && (0U != len)) {
        s_uart1_recv_cb(buf, len);
    }
}

/* 显式重启 RX DMA，为接收下一帧做好准�?*/
static void UART1_RestartRxDma(void)
{
    s_uart1_rx_frame_end = RESET;

    (void)DMA_ChCmd(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(UART1_RX_DMA_UNIT, UART1_RX_DMA_TC_FLAG);
    (void)DMA_SetDestAddr(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH, (uint32_t)s_uart1_rx_buf);
    (void)DMA_SetTransCount(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH, UART_DMA_FRAME_LEN_MAX);
    (void)DMA_SetBlockSize(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH, 1U);
    (void)DMA_ChCmd(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH, ENABLE);
}

// 接收DMA传输完成中断回调函数
static void UART1_RX_DMA_TC_IrqCallback(void)
{
    // 解决接收长度超过DMA缓冲区长度时，无法正确接收的问题，一帧结束统一放到超时中断中处�?
    // s_uart1_rx_frame_end  = SET;
    // s_uart1_rx_frame_done = 1U;
        /* 缓冲区已满，关闭RX超时功能 */
    // USART_FuncCmd(UART1_UNIT, USART_RX_TIMEOUT, DISABLE);

    s_uart1_rx_len        = UART_DMA_FRAME_LEN_MAX;
    UART1_NotifyRecv(s_uart1_rx_buf, s_uart1_rx_len);
    DMA_ClearTransCompleteStatus(UART1_RX_DMA_UNIT, UART1_RX_DMA_TC_FLAG);

    // 一帧满了，重启接收DMA
    UART1_RestartRxDma();
}

// 发送DMA传输完成中断回调函数，通知上层发送完�?
static void UART1_TX_DMA_TC_IrqCallback(void)
{
    (void)DMA_ChCmd(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, DISABLE);
    USART_FuncCmd(UART1_UNIT, USART_INT_TX_CPLT, ENABLE);

    DMA_ClearTransCompleteStatus(UART1_TX_DMA_UNIT, UART1_TX_DMA_TC_FLAG);
}

// 接收超时中断回调函数
static void UART1_RxTimeout_IrqCallback(void)
{
    if (s_uart1_rx_frame_end != SET) {
        s_uart1_rx_frame_end = SET;
        /* 实际接收字节�?= 缓冲区总大�?- DMA剩余传输计数 */
        s_uart1_rx_len = UART_DMA_FRAME_LEN_MAX -
                         (uint16_t)DMA_GetTransCount(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH);
        UART1_NotifyRecv(s_uart1_rx_buf, s_uart1_rx_len);
        /* 显式重启RX DMA，为接收下一帧做好准�?*/
        UART1_RestartRxDma();
    }

    UART1_StopTimeoutTimer();

    USART_ClearStatus(UART1_UNIT, USART_FLAG_RX_TIMEOUT);
}

// 发送完成中断回调函�?
static void UART1_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(UART1_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    s_uart1_tx_busy = RESET;
}

// 接收错误中断回调函数
static void UART1_RxError_IrqCallback(void)
{
    (void)USART_ReadData(UART1_UNIT);

    USART_ClearStatus(UART1_UNIT,
                      USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN);
}

// DMA配置函数
static int32_t UART1_DMA_Config(void)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    /* 使能 DMA1、DMA2 �?AOS 时钟 */
    UART1_RX_DMA_FCG_ENABLE();
    UART1_TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* ---- 接收DMA：DMA1 CH0，搬�?USART1_RDR �?s_uart1_rx_buf ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;                  /* 传输完成中断使能 */
    stcDmaInit.u32BlockSize  = 1UL;                             /* 每次触发�?字节 */
    stcDmaInit.u32TransCount = UART_DMA_FRAME_LEN_MAX;      /* 总搬运次�?缓冲区大�?*/
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;              /* 8位数据宽�?*/
    stcDmaInit.u32DestAddr   = (uint32_t)s_uart1_rx_buf;        /* 目标：接收缓冲区 */
    stcDmaInit.u32SrcAddr    = (uint32_t)(&UART1_UNIT->RDR);    /* 源：USART数据寄存�?*/
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;               /* 源地址固定 */
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;              /* 目标地址自增 */
    i32Ret = DMA_Init(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* 注册接收DMA传输完成中断 */
        stcIrqSignConfig.enIntSrc    = UART1_RX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn      = UART1_RX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &UART1_RX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        /* 设置DMA触发事件源（每收到一字节触发一次DMA搬运�?*/
        AOS_SetTriggerEventSrc(UART1_RX_DMA_TRIG_SEL, UART1_RX_DMA_TRIG_EVT_SRC);

        /* 使能DMA控制器、传输完成中断及通道 */
        DMA_Cmd(UART1_RX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(UART1_RX_DMA_UNIT, UART1_RX_DMA_TC_INT, ENABLE);
        (void)DMA_ChCmd(UART1_RX_DMA_UNIT, UART1_RX_DMA_CH, ENABLE);
    }

    /* ---- 发送DMA：DMA2 CH0，内存缓冲区 �?USART1_TDR（通道按需启动�?---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = UART_DMA_TX_BUF_LEN_MAX;
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr   = (uint32_t)(&UART1_UNIT->TDR); /* 目标：USART发送寄存器 */
    stcDmaInit.u32SrcAddr    = (uint32_t)s_uart1_tx_buf;        /* 源：占位，发送前重新配置 */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* 注册发送DMA传输完成中断 */
        stcIrqSignConfig.enIntSrc    = UART1_TX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn      = UART1_TX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &UART1_TX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(UART1_TX_DMA_TRIG_SEL, UART1_TX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(UART1_TX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(UART1_TX_DMA_UNIT, UART1_TX_DMA_TC_INT, ENABLE);
        /* 发送DMA通道按需使能 */
    }

    return i32Ret;
}

static uint16_t UART1_CalcTimeoutCompareValue(uint16_t timeout_bits, uint32_t clock_div)
{
    uint16_t u16Div;
    uint16_t u16Delay;
    uint16_t u16CompareValue;

    /* 根据分频系数确定同步延迟补偿�?*/
    if (TMR0_CLK_DIV1 == clock_div) {
        u16Delay = 7U;
    } else if (TMR0_CLK_DIV2 == clock_div) {
        u16Delay = 5U;
    } else if ((TMR0_CLK_DIV4  == clock_div) ||
               (TMR0_CLK_DIV8  == clock_div) ||
               (TMR0_CLK_DIV16 == clock_div)) {
        u16Delay = 3U;
    } else {
        u16Delay = 2U;
    }

    /* 计算比较值：将超时比特数换算为分频后的计数值，并减去同步延�?*/
    u16Div          = (uint16_t)1U << (clock_div >> TMR0_BCONR_CKDIVA_POS);
    u16CompareValue = ((timeout_bits + u16Div - 1U) / u16Div) - u16Delay;

    return u16CompareValue;
}

// 停止超时定时器函�?
static void UART1_StopTimeoutTimer(void)
{
    uint32_t u32ClrMask;
    uint32_t u32SetMask;
    uint32_t u32BitOffset;
    CM_TMR0_TypeDef *TMR0x = UART1_TMR0_UNIT;
    uint32_t u32Ch = UART1_TMR0_CH;

    u32BitOffset = 16UL * u32Ch;

    /* 第一步：SYNCLKA=1，SYNSA=0（切换为同步模式，停止计数）*/
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA) << u32BitOffset;
    u32SetMask =  TMR0_BCONR_SYNCLKA                     << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);

    /* 第二步：CSTA=0，SYNCLKA=0，SYNSA=1（清除启动位，恢复异步模式）*/
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA | TMR0_BCONR_CSTA) << u32BitOffset;
    u32SetMask =  TMR0_BCONR_SYNSA                                           << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);
}

// TMR0配置函数
static void UART1_TMR0_Config(uint16_t timeout_bits)
{
    stc_tmr0_init_t stcTmr0Init;

    UART1_TMR0_FCG_ENABLE();

    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_XTAL32;  /* 时钟源：XTAL32 */
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;         /* 8分频 */
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;          /* 比较功能 */
    stcTmr0Init.u16CompareValue =
        UART1_CalcTimeoutCompareValue(timeout_bits, stcTmr0Init.u32ClockDiv);
    (void)TMR0_Init(UART1_TMR0_UNIT, UART1_TMR0_CH, &stcTmr0Init);

    /* 配置硬件自动启动和自动清零（由USART RX事件触发�?*/
    TMR0_HWStartCondCmd(UART1_TMR0_UNIT, UART1_TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(UART1_TMR0_UNIT, UART1_TMR0_CH, ENABLE);
}

// UART DMA初始化函�?
int drv_uart1_init(uint32_t baudrate)
{
    stc_usart_uart_init_t   stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;
    int32_t                 i32Ret;

    if (0UL == baudrate) {
        return LL_ERR_INVD_PARAM;
    }

    i32Ret = UART1_DMA_Config();
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    UART1_TMR0_Config(UART1_TIMEOUT_BITS);

    GPIO_SetFunc(UART1_RX_PORT, UART1_RX_PIN, UART1_RX_GPIO_FUNC);
    GPIO_SetFunc(UART1_TX_PORT, UART1_TX_PIN, UART1_TX_GPIO_FUNC);

    UART1_FCG_ENABLE();

    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv      = USART_CLK_DIV64;
    stcUartInit.u32CKOutput      = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32Baudrate      = baudrate;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    i32Ret = USART_UART_Init(UART1_UNIT, &stcUartInit, NULL);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    /* 注册USART发送完成中�?*/
    stcIrqSigninConfig.enIRQn      = UART1_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc    = UART1_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &UART1_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 注册USART接收错误中断 */
    stcIrqSigninConfig.enIRQn      = UART1_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc    = UART1_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &UART1_RxError_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 注册USART接收超时中断 */
    stcIrqSigninConfig.enIRQn      = UART1_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc    = UART1_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &UART1_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 清除定时器标志位 */
    UART1_StopTimeoutTimer();
    USART_ClearStatus(UART1_UNIT, USART_FLAG_RX_TIMEOUT);

    // /* 恢复寄存器写保护，再开�?USART 接收通路 */
    // Board_PeriphLock();

    USART_FuncCmd(UART1_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT |
                               USART_INT_RX_TIMEOUT), ENABLE);

    return LL_OK;
}

// 设置串口接收回调函数
void drv_uart1_set_recv_callback(drv_uart_recv_cb_t cb)
{
    s_uart1_recv_cb = cb;
}

// 先把调用方数据复制到内部 TX 缓冲，再启动 DMA�?
int drv_uart1_send(const uint8_t *buf, uint16_t len)
{
    int32_t i32Ret;
    uint32_t spin = 0U;

    if ((NULL == buf) || (0U == len)) {
        return LL_ERR_INVD_PARAM;
    }

    if (len > UART_DMA_TX_BUF_LEN_MAX) {
        len = UART_DMA_TX_BUF_LEN_MAX;
    }

    while ((SET == s_uart1_tx_busy) && (spin < 5000000U)) {
        spin++;
    }
    if (SET == s_uart1_tx_busy) {
        return LL_ERR_TIMEOUT;
    }

    s_uart1_tx_busy = SET;
    (void)memcpy(s_uart1_tx_buf, buf, len);

    (void)DMA_ChCmd(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(UART1_TX_DMA_UNIT, UART1_TX_DMA_TC_FLAG);

    i32Ret = DMA_SetSrcAddr(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, (uint32_t)s_uart1_tx_buf);
    if (LL_OK == i32Ret) {
        i32Ret = DMA_SetTransCount(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, len);
    }
    if (LL_OK == i32Ret) {
        i32Ret = DMA_SetBlockSize(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, 1U);
    }
    if (LL_OK == i32Ret) {
        i32Ret = DMA_ChCmd(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, ENABLE);
    }
    if (LL_OK == i32Ret) {
        USART_FuncCmd(UART1_UNIT, USART_TX, ENABLE);
    } else {
        s_uart1_tx_busy = RESET;
        (void)DMA_ChCmd(UART1_TX_DMA_UNIT, UART1_TX_DMA_CH, DISABLE);
        return i32Ret;
    }

    return LL_OK;
}
