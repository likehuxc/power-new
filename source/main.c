/**
 *******************************************************************************
 * @file  usart/usart_uart_dma/source/main.c
 * @brief 本示例演示通过DMA方式实现UART数据接收与发送。
 @verbatim
   修改记录:
   日期             作者            说明
   2022-03-31       CDT             初始版本
   2022-10-31       CDT             删除冗余代码；USART溢出时读取USART_DR.RDR
   2023-01-15       CDT             更新Timer0比较值的计算公式
   2023-09-30       CDT             将USART_DR拆分为USART_RDR和USART_TDR
   2024-11-08       CDT             优化USART_TxComplete_IrqCallback；新增USART_StopTimeoutTimer
   2026-05-13       User            适配power-new工程：USART1 PA0(RX)/PA2(TX)，PB7心跳LED
                                    发送采用轮询方式(USART_UART_Trans)，接收采用DMA+TMR0超时
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
 * 头文件包含
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
 * 本地类型定义 ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * 本地宏定义 ('#define')
 ******************************************************************************/
/* 外设寄存器写使能/写保护选择 */
#define LL_PERIPH_SEL                   (LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | \
                                         LL_PERIPH_EFM | LL_PERIPH_SRAM)

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

/* 发送DMA：DMA2 通道0，触发源为 USART1_TI（预留，暂不使能通道） */
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

/* 接收缓冲区最大长度 */
#define APP_FRAME_LEN_MAX               (500U)

/* 轮询发送超时时间（ms） */
#define USART_TX_TIMEOUT_MS             (1000UL)

/* 心跳LED：PB7 */
#define HEART_LED_PORT                  (GPIO_PORT_B)
#define HEART_LED_PIN                   (GPIO_PIN_07)
#define HEART_LED_TOGGLE()              (GPIO_TogglePins(HEART_LED_PORT, HEART_LED_PIN))
#define HEART_LED_BLINK_PERIOD_MS       (1000UL)

/* 每次心跳发送的字符串 */
#define HEARTBEAT_MSG                   "AAA\r\n"

/*******************************************************************************
 * 全局变量定义（在头文件中以 extern 声明）
 ******************************************************************************/

/*******************************************************************************
 * 本地函数声明 ('static')
 ******************************************************************************/

/*******************************************************************************
 * 本地变量定义 ('static')
 ******************************************************************************/
static __IO en_flag_status_t m_enRxFrameEnd;        /* 接收帧结束标志 */
static __IO en_flag_status_t m_enTxBusy;            /* 发送忙标志 */
static __IO uint16_t         m_u16RxLen;             /* 本帧实际接收字节数 */
static uint8_t               m_au8RxBuf[APP_FRAME_LEN_MAX];   /* 接收缓冲区 */
static uint8_t               m_au8HeartbeatMsg[] = HEARTBEAT_MSG; /* 心跳发送内容 */

/*******************************************************************************
 * 函数实现 - 全局 ('extern') 与本地 ('static')
 ******************************************************************************/

/**
 * @brief  SysTick 中断服务函数，驱动软件延时计数。
 */
void SysTick_Handler(void)
{
    SysTick_IncTick();

    __DSB();
}

/**
 * @brief  初始化心跳LED（PB7，推挽输出，初始高电平熄灭）。
 */
static void HeartLed_Init(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState = PIN_STAT_SET;
    stcGpioInit.u16PinDir   = PIN_DIR_OUT;
    (void)GPIO_Init(HEART_LED_PORT, HEART_LED_PIN, &stcGpioInit);
}

/**
 * @brief  通过USART轮询方式发送数据。
 *         发送前使能TX，TX完成中断回调中关闭TX。
 * @param  [in] pu8Data     待发送数据指针
 * @param  [in] u16Len      发送字节数
 * @retval int32_t  LL_OK 或错误码
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
 * @brief  心跳处理：每1秒翻转LED并发送"AAA\r\n"。
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
 * @brief  接收DMA传输完成中断回调（缓冲区已满500字节）。
 *         置位帧结束标志，并停止RX超时计时器。
 */
static void RX_DMA_TC_IrqCallback(void)
{
    m_enRxFrameEnd = SET;
    m_u16RxLen     = APP_FRAME_LEN_MAX;

    /* 缓冲区已满，关闭RX超时功能（LLP自动重载，无需软件触发） */
    USART_FuncCmd(USART_UNIT, USART_RX_TIMEOUT, DISABLE);

    DMA_ClearTransCompleteStatus(RX_DMA_UNIT, RX_DMA_TC_FLAG);
}

/**
 * @brief  发送DMA传输完成中断回调（预留，目前TX采用轮询）。
 *         使能USART TX完成中断，由TX完成回调负责关闭TX。
 */
static void TX_DMA_TC_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, USART_INT_TX_CPLT, ENABLE);

    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);
}

/**
 * @brief  初始化DMA（接收DMA1/CH0 + 发送DMA2/CH0）。
 * @retval int32_t  LL_OK 表示成功，LL_ERR_INVD_PARAM 表示参数错误。
 */
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

    /* ---- 发送DMA：DMA2 CH0，m_au8RxBuf → USART1_TDR（预留，通道暂不使能） ---- */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8RxBuf);
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr   = (uint32_t)(&USART_UNIT->TDR); /* 目标：USART发送寄存器 */
    stcDmaInit.u32SrcAddr    = (uint32_t)m_au8RxBuf;         /* 源：接收缓冲区（echo用） */
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
        /* 发送DMA通道按需使能（需要echo时再调用DMA_ChCmd） */
    }

    return i32Ret;
}

/**
 * @brief  配置TMR0用于USART接收超时检测。
 * @param  [in] u16TimeoutBits  超时对应的比特位数
 */
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

/**
 * @brief  强制停止TMR0超时计时器。
 *         在DMA缓冲区满（非超时）结束帧时调用，避免计时器残留触发。
 * @param  [in] TMR0x   TMR0外设基地址
 * @param  [in] u32Ch   TMR0通道
 */
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

/**
 * @brief  USART接收超时中断回调。
 *         帧间隙超时表示一帧数据接收完毕，记录实际长度并通过AOS触发DMA重载。
 */
static void USART_RxTimeout_IrqCallback(void)
{
    if (m_enRxFrameEnd != SET) {
        m_enRxFrameEnd = SET;
        /* 实际接收字节数 = 缓冲区总大小 - DMA剩余传输计数 */
        m_u16RxLen = APP_FRAME_LEN_MAX - (uint16_t)DMA_GetTransCount(RX_DMA_UNIT, RX_DMA_CH);

        /* 软件触发AOS，通过LLP重载DMA，为接收下一帧做好准备 */
        AOS_SW_Trigger();
    }

    USART_StopTimeoutTimer(TMR0_UNIT, TMR0_CH);

    USART_ClearStatus(USART_UNIT, USART_FLAG_RX_TIMEOUT);
}

/**
 * @brief  USART发送完成中断回调。
 *         轮询发送结束后，关闭TX发送器及TX完成中断，清除忙标志。
 */
static void USART_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    m_enTxBusy = RESET;
}

/**
 * @brief  USART接收错误中断回调。
 *         读取RDR清除错误，并清除奇偶校验/帧错误/溢出标志。
 */
static void USART_RxError_IrqCallback(void)
{
    (void)USART_ReadData(USART_UNIT);

    USART_ClearStatus(USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

/**
 * @brief  主函数
 * @param  无
 * @retval int32_t
 */
int32_t main(void)
{
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;

    /* 解除外设寄存器写保护 */
    LL_PERIPH_WE(LL_PERIPH_SEL);

    /* 初始化BSP系统时钟 */
    BSP_CLK_Init();

    /* 初始化SysTick，1ms节拍，供心跳计时使用 */
    (void)SysTick_Init(1000U);

    /* 初始化心跳LED（PB7） */
    HeartLed_Init();

    /* 初始化DMA（接收DMA1/CH0 + 发送DMA2/CH0） */
    (void)DMA_Config();

    /* 初始化TMR0，用于检测RX帧间隙超时 */
    TMR0_Config(USART_TIMEOUT_BITS);

    /* 配置USART收发引脚复用功能 */
    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    /* 使能USART外设时钟 */
    USART_FCG_ENABLE();

    /* 初始化UART（115200bps，8bit，1停止位，无校验） */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv      = USART_CLK_DIV64;
    stcUartInit.u32Baudrate      = USART_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(USART_UNIT, &stcUartInit, NULL)) {
        /* 初始化失败，停在此处并持续闪烁心跳LED */
        for (;;) {
            HeartLed_Process();
        }
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

    /* 恢复外设寄存器写保护 */
    LL_PERIPH_WP(LL_PERIPH_SEL);

    /* 使能接收、接收中断、接收超时功能 */
    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT |
                               USART_INT_RX_TIMEOUT), ENABLE);

    for (;;) {
        /* 心跳：每1秒翻转LED并发送"AAA\r\n" */
        HeartLed_Process();

        /* 检测是否有完整帧接收完成 */
        if (SET == m_enRxFrameEnd) {
            m_enRxFrameEnd = RESET;
            /* m_au8RxBuf[0 .. m_u16RxLen-1] 为本帧数据，在此处理业务逻辑 */
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
