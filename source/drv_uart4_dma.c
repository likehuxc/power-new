/**
 *******************************************************************************
 * @file  drv_uart4_dma.c
 * @brief USART4 DMA 驱动
 *******************************************************************************
 */

#include "drv_uart_dma.h"

#include <stddef.h>
#include <string.h>

#include "hc32_ll.h"

/* 接收DMA：DMA2 通道1，触发源为 USART4_RI */
#define UART4_RX_DMA_UNIT               (CM_DMA2)
#define UART4_RX_DMA_CH                 (DMA_CH1)
#define UART4_RX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define UART4_RX_DMA_TRIG_SEL           (AOS_DMA2_1)
#define UART4_RX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART4_RI)
#define UART4_RX_DMA_TC_INT             (DMA_INT_TC_CH1)
#define UART4_RX_DMA_TC_FLAG            (DMA_FLAG_TC_CH1)
#define UART4_RX_DMA_TC_IRQn            (INT005_IRQn)
#define UART4_RX_DMA_TC_INT_SRC         (INT_SRC_DMA2_TC1)

/* 发送DMA：DMA1 通道1，触发源为 USART4_TI */
#define UART4_TX_DMA_UNIT               (CM_DMA1)
#define UART4_TX_DMA_CH                 (DMA_CH1)
#define UART4_TX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define UART4_TX_DMA_TRIG_SEL           (AOS_DMA1_1)
#define UART4_TX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART4_TI)
#define UART4_TX_DMA_TC_INT             (DMA_INT_TC_CH1)
#define UART4_TX_DMA_TC_FLAG            (DMA_FLAG_TC_CH1)
#define UART4_TX_DMA_TC_IRQn            (INT006_IRQn)
#define UART4_TX_DMA_TC_INT_SRC         (INT_SRC_DMA1_TC1)

/* TMR0：用于USART4接收超时检测 */
#define UART4_TMR0_UNIT                 (CM_TMR0_2)
#define UART4_TMR0_CH                   (TMR0_CH_B)
#define UART4_TMR0_FCG_ENABLE()         (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_2, ENABLE))

/* USART4 收发引脚定义（数据手册：Func36=USART4_TX，Func37=USART4_RX） */
#define UART4_RX_PORT                   (GPIO_PORT_D)   /* PD8：USART4_RX */
#define UART4_RX_PIN                    (GPIO_PIN_08)
#define UART4_RX_GPIO_FUNC              (GPIO_FUNC_37)

#define UART4_TX_PORT                   (GPIO_PORT_D)   /* PD9：USART4_TX */
#define UART4_TX_PIN                    (GPIO_PIN_09)
#define UART4_TX_GPIO_FUNC              (GPIO_FUNC_36)

/* USART4 外设单元 */
#define UART4_UNIT                      (CM_USART4)
#define UART4_FCG_ENABLE()              (FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART4, ENABLE))

/* USART4 中断定义 */
#define UART4_TX_CPLT_IRQn              (INT007_IRQn)
#define UART4_TX_CPLT_INT_SRC           (INT_SRC_USART4_TCI)

#define UART4_RX_ERR_IRQn               (INT008_IRQn)
#define UART4_RX_ERR_INT_SRC            (INT_SRC_USART4_EI)

#define UART4_RX_TIMEOUT_IRQn           (INT009_IRQn)
#define UART4_RX_TIMEOUT_INT_SRC        (INT_SRC_USART4_RTO)

/* 接收超时位数（帧间隙超过此位时间则判定帧结束） */
#define UART4_TIMEOUT_BITS              (2000U)

static __IO en_flag_status_t s_uart4_rx_frame_end;        /* 接收帧结束标志 */
static __IO en_flag_status_t s_uart4_tx_busy;            /* 发送忙标志 */
static __IO uint16_t         s_uart4_rx_len;            /* 本帧实际接收字节数 */
static uint8_t               s_uart4_rx_buf[DRV_UART_DMA_FRAME_LEN_MAX] = {0};   /* 接收缓冲区 */
static uint8_t               s_uart4_tx_buf[DRV_UART_DMA_TX_BUF_LEN_MAX] = {0};   /* 发送缓冲区 */
static drv_uart_recv_cb_t    s_uart4_recv_cb;            /* 接收回调函数 */

static void UART4_StopTimeoutTimer(void);
static uint16_t UART4_CalcTimeoutCompareValue(uint16_t timeout_bits, uint32_t clock_div);
static void UART4_RestartRxDma(void);
static void UART4_NotifyRecv(const uint8_t *buf, uint16_t len);
static int32_t UART4_DMA_Config(void);
static void UART4_TMR0_Config(uint16_t timeout_bits);
static void UART4_RX_DMA_TC_IrqCallback(void);
static void UART4_TX_DMA_TC_IrqCallback(void);
static void UART4_RxTimeout_IrqCallback(void);
static void UART4_TxComplete_IrqCallback(void);
static void UART4_RxError_IrqCallback(void);

static void UART4_NotifyRecv(const uint8_t *buf, uint16_t len)
{
    if ((NULL != s_uart4_recv_cb) && (NULL != buf) && (0U != len)) {
        s_uart4_recv_cb(buf, len);
    }
}

/* 显式重启 RX DMA，为接收下一帧做好准备 */
static void UART4_RestartRxDma(void)
{
    s_uart4_rx_frame_end = RESET;

    (void)DMA_ChCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(UART4_RX_DMA_UNIT, UART4_RX_DMA_TC_FLAG);
    (void)DMA_SetDestAddr(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, (uint32_t)s_uart4_rx_buf);
    (void)DMA_SetTransCount(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, DRV_UART_DMA_FRAME_LEN_MAX);
    (void)DMA_SetBlockSize(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, 1U);
    (void)DMA_ChCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, ENABLE);
}

// 接收DMA传输完成中断回调函数
static void UART4_RX_DMA_TC_IrqCallback(void)
{
    // 解决接收长度超过DMA缓冲区长度时，无法正确接收的问题，一帧结束统一放到超时中断中处理
    // s_uart4_rx_frame_end  = SET;
    // s_uart4_rx_frame_done = 1U;
        /* 缓冲区已满，关闭RX超时功能 */
    // USART_FuncCmd(UART4_UNIT, USART_RX_TIMEOUT, DISABLE);

    s_uart4_rx_len       = DRV_UART_DMA_FRAME_LEN_MAX;
    UART4_NotifyRecv(s_uart4_rx_buf, s_uart4_rx_len);

    DMA_ClearTransCompleteStatus(UART4_RX_DMA_UNIT, UART4_RX_DMA_TC_FLAG);

    // 一帧满了，重启接收DMA
    UART4_RestartRxDma();
}

// 发送DMA传输完成中断回调函数，通知上层发送完成
static void UART4_TX_DMA_TC_IrqCallback(void)
{
    (void)DMA_ChCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, DISABLE);
    /* DMA TC 只表示数据已搬到 USART TDR，需等 USART TCI 后再清 busy。 */
    USART_FuncCmd(UART4_UNIT, USART_INT_TX_CPLT, ENABLE);

    DMA_ClearTransCompleteStatus(UART4_TX_DMA_UNIT, UART4_TX_DMA_TC_FLAG);
}

// 接收超时中断回调函数
static void UART4_RxTimeout_IrqCallback(void)
{
    if (s_uart4_rx_frame_end != SET) {
        s_uart4_rx_frame_end = SET;
        /* 实际接收字节数 = 缓冲区总大小 - DMA剩余传输计数 */
        s_uart4_rx_len = DRV_UART_DMA_FRAME_LEN_MAX -
                         (uint16_t)DMA_GetTransCount(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH);
        UART4_NotifyRecv(s_uart4_rx_buf, s_uart4_rx_len);
        /* 显式重启RX DMA，为接收下一帧做好准备 */
        UART4_RestartRxDma();
    }

    UART4_StopTimeoutTimer();

    USART_ClearStatus(UART4_UNIT, USART_FLAG_RX_TIMEOUT);
}

// 发送完成中断回调函数
static void UART4_TxComplete_IrqCallback(void)
{
    /* TCI 表示最后一位已经移出发送移位寄存器，此时内部 TX 缓冲可复用。 */
    USART_FuncCmd(UART4_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    s_uart4_tx_busy = RESET;
}

// 接收错误中断回调函数
static void UART4_RxError_IrqCallback(void)
{
    (void)USART_ReadData(UART4_UNIT);

    USART_ClearStatus(UART4_UNIT,
                      USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN);
}

// DMA配置函数
static int32_t UART4_DMA_Config(void)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    /* 使能 DMA1、DMA2 及 AOS 时钟 */
    UART4_RX_DMA_FCG_ENABLE();
    UART4_TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* ---- 接收DMA：DMA2 CH1，搬运 USART4_RDR → s_uart4_rx_buf ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;                  /* 传输完成中断使能 */
    stcDmaInit.u32BlockSize  = 1UL;                             /* 每次触发搬1字节 */
    stcDmaInit.u32TransCount = DRV_UART_DMA_FRAME_LEN_MAX;      /* 总搬运次数=缓冲区大小 */
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;              /* 8位数据宽度 */
    stcDmaInit.u32DestAddr   = (uint32_t)s_uart4_rx_buf;        /* 目标：接收缓冲区 */
    stcDmaInit.u32SrcAddr    = (uint32_t)(&UART4_UNIT->RDR);    /* 源：USART数据寄存器 */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;               /* 源地址固定 */
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;              /* 目标地址自增 */
    i32Ret = DMA_Init(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* 注册接收DMA传输完成中断 */
        stcIrqSignConfig.enIntSrc    = UART4_RX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn      = UART4_RX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &UART4_RX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        /* 设置DMA触发事件源（每收到一字节触发一次DMA搬运） */
        AOS_SetTriggerEventSrc(UART4_RX_DMA_TRIG_SEL, UART4_RX_DMA_TRIG_EVT_SRC);

        /* 使能DMA控制器、传输完成中断及通道 */
        DMA_Cmd(UART4_RX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_TC_INT, ENABLE);
        (void)DMA_ChCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, ENABLE);
    }

    /* ---- 发送DMA：DMA1 CH1，内存缓冲区 → USART4_TDR（通道按需启动） ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = DRV_UART_DMA_TX_BUF_LEN_MAX;
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr   = (uint32_t)(&UART4_UNIT->TDR); /* 目标：USART发送寄存器 */
    stcDmaInit.u32SrcAddr    = (uint32_t)s_uart4_tx_buf;        /* 源：占位，发送前重新配置 */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* 注册发送DMA传输完成中断 */
        stcIrqSignConfig.enIntSrc    = UART4_TX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn      = UART4_TX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &UART4_TX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(UART4_TX_DMA_TRIG_SEL, UART4_TX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(UART4_TX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_TC_INT, ENABLE);
        /* 发送DMA通道按需使能 */
    }

    return i32Ret;
}

static uint16_t UART4_CalcTimeoutCompareValue(uint16_t timeout_bits, uint32_t clock_div)
{
    uint16_t u16Div;
    uint16_t u16Delay;
    uint16_t u16CompareValue;

    /* 根据分频系数确定同步延迟补偿值 */
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

    /* 计算比较值：将超时比特数换算为分频后的计数值，并减去同步延迟 */
    u16Div          = (uint16_t)1U << (clock_div >> TMR0_BCONR_CKDIVA_POS);
    u16CompareValue = ((timeout_bits + u16Div - 1U) / u16Div) - u16Delay;

    return u16CompareValue;
}

// 停止超时定时器函数
static void UART4_StopTimeoutTimer(void)
{
    uint32_t u32ClrMask;
    uint32_t u32SetMask;
    uint32_t u32BitOffset;
    CM_TMR0_TypeDef *TMR0x = UART4_TMR0_UNIT;
    uint32_t u32Ch = UART4_TMR0_CH;

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
static void UART4_TMR0_Config(uint16_t timeout_bits)
{
    stc_tmr0_init_t stcTmr0Init;

    UART4_TMR0_FCG_ENABLE();

    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_XTAL32;  /* 时钟源：XTAL32 */
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;         /* 8分频 */
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;          /* 比较功能 */
    stcTmr0Init.u16CompareValue =
        UART4_CalcTimeoutCompareValue(timeout_bits, stcTmr0Init.u32ClockDiv);
    (void)TMR0_Init(UART4_TMR0_UNIT, UART4_TMR0_CH, &stcTmr0Init);

    /* 配置硬件自动启动和自动清零（由USART RX事件触发） */
    TMR0_HWStartCondCmd(UART4_TMR0_UNIT, UART4_TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(UART4_TMR0_UNIT, UART4_TMR0_CH, ENABLE);
}

// UART DMA初始化函数
int drv_uart4_init(uint32_t baudrate)
{
    stc_usart_uart_init_t   stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;
    int32_t                 i32Ret;

    if (0UL == baudrate) {
        return LL_ERR_INVD_PARAM;
    }

    i32Ret = UART4_DMA_Config();
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    UART4_TMR0_Config(UART4_TIMEOUT_BITS);

    GPIO_SetFunc(UART4_RX_PORT, UART4_RX_PIN, UART4_RX_GPIO_FUNC);
    GPIO_SetFunc(UART4_TX_PORT, UART4_TX_PIN, UART4_TX_GPIO_FUNC);

    UART4_FCG_ENABLE();

    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv      = USART_CLK_DIV64;
    stcUartInit.u32CKOutput      = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32Baudrate      = baudrate;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    i32Ret = USART_UART_Init(UART4_UNIT, &stcUartInit, NULL);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    /* 注册USART发送完成中断 */
    stcIrqSigninConfig.enIRQn      = UART4_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc    = UART4_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &UART4_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 注册USART接收错误中断 */
    stcIrqSigninConfig.enIRQn      = UART4_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc    = UART4_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &UART4_RxError_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 注册USART接收超时中断 */
    stcIrqSigninConfig.enIRQn      = UART4_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc    = UART4_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &UART4_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 清除定时器标志位 */
    UART4_StopTimeoutTimer();
    USART_ClearStatus(UART4_UNIT, USART_FLAG_RX_TIMEOUT);

    USART_FuncCmd(UART4_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT |
                               USART_INT_RX_TIMEOUT), ENABLE);

    return LL_OK;
}

// 设置串口接收回调函数
void drv_uart4_set_recv_callback(drv_uart_recv_cb_t cb)
{
    s_uart4_recv_cb = cb;
}

// 先把调用方数据复制到内部 TX 缓冲，再启动 DMA。
int drv_uart4_send(const uint8_t *buf, uint16_t len)
{
    int32_t i32Ret;
    uint32_t spin = 0U;

    if ((NULL == buf) || (0U == len)) {
        return LL_ERR_INVD_PARAM;
    }

    if (len > DRV_UART_DMA_TX_BUF_LEN_MAX) {
        len = DRV_UART_DMA_TX_BUF_LEN_MAX;
    }

    while ((SET == s_uart4_tx_busy) && (spin < 5000000U)) {
        spin++;
    }
    if (SET == s_uart4_tx_busy) {
        return LL_ERR_TIMEOUT;
    }

    s_uart4_tx_busy = SET;
    (void)memcpy(s_uart4_tx_buf, buf, len);

    (void)DMA_ChCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(UART4_TX_DMA_UNIT, UART4_TX_DMA_TC_FLAG);

    i32Ret = DMA_SetSrcAddr(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, (uint32_t)s_uart4_tx_buf);
    if (LL_OK == i32Ret) {
        i32Ret = DMA_SetTransCount(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, len);
    }
    if (LL_OK == i32Ret) {
        i32Ret = DMA_SetBlockSize(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, 1U);
    }
    if (LL_OK == i32Ret) {
        i32Ret = DMA_ChCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, ENABLE);
    }
    if (LL_OK == i32Ret) {
        USART_FuncCmd(UART4_UNIT, USART_TX, ENABLE);
    } else {
        s_uart4_tx_busy = RESET;
        (void)DMA_ChCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, DISABLE);
        return i32Ret;
    }

    return LL_OK;
}
