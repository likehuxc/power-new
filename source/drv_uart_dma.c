/**
 *******************************************************************************
 * @file  drv_uart_dma.c
 *******************************************************************************
 */

#include "drv_uart_dma.h"

#include <string.h>

#include "board.h"
#include "hc32_ll.h"

/* 接收DMA：DMA1 通道0，触发源为 USART1_RI */
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

/* 发送DMA：DMA2 通道0，触发源为 USART1_TI */
#define TX_DMA_UNIT                     (CM_DMA2)
#define TX_DMA_CH                       (DMA_CH0)
#define TX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define TX_DMA_TRIG_SEL                 (AOS_DMA2_0)
#define TX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART1_TI)
#define TX_DMA_TC_INT                   (DMA_INT_TC_CH0)
#define TX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH0)
#define TX_DMA_TC_IRQn                  (INT001_IRQn)
#define TX_DMA_TC_INT_SRC               (INT_SRC_DMA2_TC0)

/* TMR0：用于USART接收超时检测 */
#define TMR0_UNIT                       (CM_TMR0_1)
#define TMR0_CH                         (TMR0_CH_A)
#define TMR0_FCG_ENABLE()               (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE))

/* USART 收发引脚定义（数据手册：Func32=USART1_TX，Func33=USART1_RX） */
#define USART_RX_PORT                   (GPIO_PORT_A)   /* PA0：USART1_RX */
#define USART_RX_PIN                    (GPIO_PIN_00)
#define USART_RX_GPIO_FUNC              (GPIO_FUNC_33)

#define USART_TX_PORT                   (GPIO_PORT_A)   /* PA2：USART1_TX */
#define USART_TX_PIN                    (GPIO_PIN_02)
#define USART_TX_GPIO_FUNC              (GPIO_FUNC_32)

/* USART 外设单元 */
#define USART_UNIT                      (CM_USART1)
#define USART_FCG_ENABLE()              (FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE))

/* USART 中断定义 */
#define USART_TX_CPLT_IRQn              (INT002_IRQn)
#define USART_TX_CPLT_INT_SRC           (INT_SRC_USART1_TCI)

#define USART_RX_ERR_IRQn               (INT003_IRQn)
#define USART_RX_ERR_INT_SRC            (INT_SRC_USART1_EI)

#define USART_RX_TIMEOUT_IRQn           (INT004_IRQn)
#define USART_RX_TIMEOUT_INT_SRC        (INT_SRC_USART1_RTO)

/* 波特率 */
#define USART_BAUDRATE                  (115200UL)

/* 接收超时位数（帧间隙超过此位时间则判定帧结束） */
#define USART_TIMEOUT_BITS              (2000U)

static __IO en_flag_status_t m_enRxFrameEnd;        /* 接收帧结束标志 */
static __IO en_flag_status_t m_enTxBusy;            /* 发送忙标志 */
static __IO uint16_t         m_u16RxLen;            /* 本帧实际接收字节数 */
static uint8_t               m_au8RxBuf[DRV_UART_DMA_FRAME_LEN_MAX];   /* 接收缓冲区 */

static void RX_DMA_TC_IrqCallback(void);
static void TX_DMA_TC_IrqCallback(void);
static int32_t DMA_Config(void);
static void TMR0_Config(uint16_t u16TimeoutBits);
static void USART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch);
static void USART_RxTimeout_IrqCallback(void);
static void USART_TxComplete_IrqCallback(void);
static void USART_RxError_IrqCallback(void);

int32_t DrvUartDma_Send(const uint8_t *pu8Data, uint16_t u16Len)
{
    int32_t i32Ret;

    if ((NULL == pu8Data) || (0U == u16Len)) {
        return LL_ERR_INVD_PARAM;
    }

    if (SET == m_enTxBusy) {
        return LL_ERR_BUSY;
    }

    m_enTxBusy = SET;

    (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);

    i32Ret = DMA_SetSrcAddr(TX_DMA_UNIT, TX_DMA_CH, (uint32_t)pu8Data);
    if (LL_OK == i32Ret) {
        i32Ret = DMA_SetTransCount(TX_DMA_UNIT, TX_DMA_CH, u16Len);
    }
    if (LL_OK == i32Ret) {
        i32Ret = DMA_SetBlockSize(TX_DMA_UNIT, TX_DMA_CH, 1U);
    }
    if (LL_OK == i32Ret) {
        i32Ret = DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, ENABLE);
    }
    if (LL_OK == i32Ret) {
        USART_FuncCmd(USART_UNIT, USART_TX, ENABLE);
    } else {
        m_enTxBusy = RESET;
        (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    }

    return i32Ret;
}

bool DrvUartDma_ReadFrame(uint8_t *pu8Buf, uint16_t u16BufSize, uint16_t *pu16Len)
{
    if ((NULL == pu8Buf) || (NULL == pu16Len)) {
        return false;
    }

    if (SET != m_enRxFrameEnd) {
        return false;
    }

    m_enRxFrameEnd = RESET;

    if (m_u16RxLen > u16BufSize) {
        *pu16Len = u16BufSize;
    } else {
        *pu16Len = m_u16RxLen;
    }

    (void)memcpy(pu8Buf, m_au8RxBuf, *pu16Len);
    return true;
}

// 接收DMA传输完成中断回调函数
static void RX_DMA_TC_IrqCallback(void)
{
    m_enRxFrameEnd = SET;
    m_u16RxLen     = DRV_UART_DMA_FRAME_LEN_MAX;

    /* 缓冲区已满，关闭RX超时功能（LLP自动重载，无需软件触发） */
    USART_FuncCmd(USART_UNIT, USART_RX_TIMEOUT, DISABLE);

    DMA_ClearTransCompleteStatus(RX_DMA_UNIT, RX_DMA_TC_FLAG);
}

// 发送DMA传输完成中断回调函数
static void TX_DMA_TC_IrqCallback(void)
{
    (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    USART_FuncCmd(USART_UNIT, USART_INT_TX_CPLT, ENABLE);

    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);
}

// 接收超时中断回调函数
static void USART_RxTimeout_IrqCallback(void)
{
    if (m_enRxFrameEnd != SET) {
        m_enRxFrameEnd = SET;
        /* 实际接收字节数 = 缓冲区总大小 - DMA剩余传输计数 */
        m_u16RxLen = DRV_UART_DMA_FRAME_LEN_MAX - (uint16_t)DMA_GetTransCount(RX_DMA_UNIT, RX_DMA_CH);

        /* 软件触发AOS，通过LLP重载DMA，为接收下一帧做好准备 */
        AOS_SW_Trigger();
    }

    USART_StopTimeoutTimer(TMR0_UNIT, TMR0_CH);

    USART_ClearStatus(USART_UNIT, USART_FLAG_RX_TIMEOUT);
}

// 发送完成中断回调函数
static void USART_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    m_enTxBusy = RESET;
}

// 接收错误中断回调函数
static void USART_RxError_IrqCallback(void)
{
    (void)USART_ReadData(USART_UNIT);

    USART_ClearStatus(USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

// DMA配置函数
static int32_t DMA_Config(void)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    stc_dma_llp_init_t stcDmaLlpInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    static stc_dma_llp_descriptor_t stcLlpDesc;    /* LLP描述符，需静态分配保证地址有效 */

    /* 使能 DMA1、DMA2 及 AOS 时钟 */
    RX_DMA_FCG_ENABLE();
    TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* ---- 接收DMA：DMA1 CH0，搬运 USART1_RDR → m_au8RxBuf ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;          /* 传输完成中断使能 */
    stcDmaInit.u32BlockSize  = 1UL;                     /* 每次触发搬1字节 */
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8RxBuf);   /* 总搬运次数=缓冲区大小 */
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;      /* 8位数据宽度 */
    stcDmaInit.u32DestAddr   = (uint32_t)m_au8RxBuf;   /* 目标：接收缓冲区 */
    stcDmaInit.u32SrcAddr    = (uint32_t)(&USART_UNIT->RDR); /* 源：USART数据寄存器 */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;      /* 源地址固定 */
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;     /* 目标地址自增 */
    i32Ret = DMA_Init(RX_DMA_UNIT, RX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* 配置LLP（链表指针），使DMA传输完后通过AOS软件触发自动重载，准备接收下一帧 */
        (void)DMA_LlpStructInit(&stcDmaLlpInit);
        stcDmaLlpInit.u32State = DMA_LLP_ENABLE;
        stcDmaLlpInit.u32Mode  = DMA_LLP_WAIT;
        stcDmaLlpInit.u32Addr  = (uint32_t)&stcLlpDesc;
        (void)DMA_LlpInit(RX_DMA_UNIT, RX_DMA_CH, &stcDmaLlpInit);

        /* 填写LLP描述符（指向自身，实现循环重载） */
        stcLlpDesc.SARx   = stcDmaInit.u32SrcAddr;
        stcLlpDesc.DARx   = stcDmaInit.u32DestAddr;
        stcLlpDesc.DTCTLx = (stcDmaInit.u32TransCount << DMA_DTCTL_CNT_POS) |
                            (stcDmaInit.u32BlockSize   << DMA_DTCTL_BLKSIZE_POS);
        stcLlpDesc.LLPx   = (uint32_t)&stcLlpDesc;     /* 自指，形成循环 */
        stcLlpDesc.CHCTLx = stcDmaInit.u32SrcAddrInc | stcDmaInit.u32DestAddrInc |
                            stcDmaInit.u32DataWidth   | stcDmaInit.u32IntEn       |
                            stcDmaLlpInit.u32State    | stcDmaLlpInit.u32Mode;

        /* 使能DMA重配置及LLP重配置，重配置触发源为AOS软件触发 */
        DMA_ReconfigLlpCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
        DMA_ReconfigCmd(RX_DMA_UNIT, ENABLE);
        AOS_SetTriggerEventSrc(RX_DMA_RECONF_TRIG_SEL, RX_DMA_RECONF_TRIG_EVT_SRC);

        /* 注册接收DMA传输完成中断 */
        stcIrqSignConfig.enIntSrc    = RX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn      = RX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &RX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        /* 设置DMA触发事件源（每收到一字节触发一次DMA搬运） */
        AOS_SetTriggerEventSrc(RX_DMA_TRIG_SEL, RX_DMA_TRIG_EVT_SRC);

        /* 使能DMA控制器、传输完成中断及通道 */
        DMA_Cmd(RX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(RX_DMA_UNIT, RX_DMA_TC_INT, ENABLE);
        (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
    }

    /* ---- 发送DMA：DMA2 CH0，内存缓冲区 → USART1_TDR（通道按需启动） ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8RxBuf);
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr   = (uint32_t)(&USART_UNIT->TDR); /* 目标：USART发送寄存器 */
    stcDmaInit.u32SrcAddr    = (uint32_t)m_au8RxBuf;        /* 源：占位，发送前重新配置 */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(TX_DMA_UNIT, TX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* 注册发送DMA传输完成中断 */
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
        /* 发送DMA通道按需使能 */
    }

    return i32Ret;
}

// TMR0配置函数
static void TMR0_Config(uint16_t u16TimeoutBits)
{
    uint16_t u16Div;
    uint16_t u16Delay;
    uint16_t u16CompareValue;
    stc_tmr0_init_t stcTmr0Init;

    TMR0_FCG_ENABLE();

    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_XTAL32;  /* 时钟源：XTAL32 */
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;         /* 8分频 */
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;          /* 比较功能 */

    /* 根据分频系数确定同步延迟补偿值 */
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

    /* 计算比较值：将超时比特数换算为分频后的计数值，并减去同步延迟 */
    u16Div          = (uint16_t)1U << (stcTmr0Init.u32ClockDiv >> TMR0_BCONR_CKDIVA_POS);
    u16CompareValue = ((u16TimeoutBits + u16Div - 1U) / u16Div) - u16Delay;
    stcTmr0Init.u16CompareValue = u16CompareValue;
    (void)TMR0_Init(TMR0_UNIT, TMR0_CH, &stcTmr0Init);

    /* 配置硬件自动启动和自动清零（由USART RX事件触发） */
    TMR0_HWStartCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
}

// 停止超时定时器函数
static void USART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch)
{
    uint32_t u32ClrMask;
    uint32_t u32SetMask;
    uint32_t u32BitOffset;

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

// UART DMA初始化函数
int32_t DrvUartDma_Init(void)
{
    stc_usart_uart_init_t   stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;
    int32_t                 i32Ret;

    (void)DMA_Config();

    TMR0_Config(USART_TIMEOUT_BITS);

    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    USART_FCG_ENABLE();

    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv      = USART_CLK_DIV64;
    stcUartInit.u32Baudrate      = USART_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    i32Ret = USART_UART_Init(USART_UNIT, &stcUartInit, NULL);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    /* 注册USART发送完成中断 */
    stcIrqSigninConfig.enIRQn      = USART_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc    = USART_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 注册USART接收错误中断 */
    stcIrqSigninConfig.enIRQn      = USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc    = USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxError_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 注册USART接收超时中断 */
    stcIrqSigninConfig.enIRQn      = USART_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc    = USART_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 与原版一致：先恢复寄存器写保护，再开启 USART 接收通路 */
    Board_PeriphLock();

    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT |
                               USART_INT_RX_TIMEOUT), ENABLE);

    return LL_OK;
}
