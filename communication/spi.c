/**
 *******************************************************************************
 * @file  spi.c
 * @brief 硬件 SPI 主机驱动（DMA 模式，软件 CS）
 *        PD8=MOSI(func40), PD9=MISO(func41), PD10=CS(软件), PD11=SCK(func43)
 *        DMA1 CH2=TX, CH3=RX（CH0/CH1 已被 UART 占用）
 *******************************************************************************
 */

#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"

/*******************************************************************************
 * 硬件资源定义
 ******************************************************************************/
/* SPI 外设 */
#define SPI_UNIT        (CM_SPI1)
#define SPI_CLK         (FCG1_PERIPH_SPI1)
#define SPI_TX_EVT      (EVT_SRC_SPI1_SPTI)
#define SPI_RX_EVT      (EVT_SRC_SPI1_SPRI)

/* GPIO 引脚 */
#define SPI_CS_PORT     (GPIO_PORT_D)
#define SPI_CS_PIN      (GPIO_PIN_10)
#define SPI_SCK_PORT    (GPIO_PORT_D)
#define SPI_SCK_PIN     (GPIO_PIN_11)
#define SPI_MOSI_PORT   (GPIO_PORT_D)
#define SPI_MOSI_PIN    (GPIO_PIN_08)
#define SPI_MISO_PORT   (GPIO_PORT_D)
#define SPI_MISO_PIN    (GPIO_PIN_09)

/* DMA 通道 */
#define DMA_UNIT        (CM_DMA1)
#define DMA_TX_CH       (DMA_CH2)
#define DMA_TX_TRIG     (AOS_DMA1_2)
#define DMA_RX_CH       (DMA_CH3)
#define DMA_RX_TRIG     (AOS_DMA1_3)
#define DMA_RX_INT_CH   (DMA_INT_TC_CH3)
#define DMA_RX_INT_SRC  (INT_SRC_DMA1_TC3)
#define DMA_RX_IRQn     (INT007_IRQn)

/*******************************************************************************
 * 内部变量
 ******************************************************************************/
static SemaphoreHandle_t s_xfer_done;
static uint8_t s_dummy = 0xFFU;   /* TX/RX 占位字节 */

/*******************************************************************************
 * 内部函数
 ******************************************************************************/
static void spi_cs_low(void)  { GPIO_ResetPins(SPI_CS_PORT, SPI_CS_PIN); }
static void spi_cs_high(void) { GPIO_SetPins(SPI_CS_PORT, SPI_CS_PIN); }

/**
 * @brief  DMA RX 传输完成中断回调
 */
static void dma_rx_irq_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH);
    xSemaphoreGiveFromISR(s_xfer_done, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  启动一次 DMA 传输并等待完成
 * @param  pu8TxBuf  发送缓冲区，NULL 时发送 0xFF
 * @param  pu8RxBuf  接收缓冲区，NULL 时丢弃
 * @param  u32Len    字节数
 */
static void spi_dma_transfer(const uint8_t *pu8TxBuf, uint8_t *pu8RxBuf, uint32_t u32Len)
{
    /* 配置 TX：有数据则地址递增，否则固定发 dummy */
    DMA_SetSrcAddr(DMA_UNIT, DMA_TX_CH,
                   (pu8TxBuf != NULL) ? (uint32_t)pu8TxBuf : (uint32_t)&s_dummy);
    DMA_SetTransCount(DMA_UNIT, DMA_TX_CH, (uint16_t)u32Len);

    /* 配置 RX：有缓冲区则地址递增，否则固定写 dummy */
    DMA_SetDestAddr(DMA_UNIT, DMA_RX_CH,
                    (pu8RxBuf != NULL) ? (uint32_t)pu8RxBuf : (uint32_t)&s_dummy);
    DMA_SetTransCount(DMA_UNIT, DMA_RX_CH, (uint16_t)u32Len);

    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, ENABLE);
    DMA_ChCmd(DMA_UNIT, DMA_RX_CH, ENABLE);
    SPI_Cmd(SPI_UNIT, ENABLE);

    xSemaphoreTake(s_xfer_done, portMAX_DELAY);

    SPI_Cmd(SPI_UNIT, DISABLE);
}

/*******************************************************************************
 * 公共接口
 ******************************************************************************/

/**
 * @brief  初始化 SPI1 及 DMA 通道
 */
void spi_init(void)
{
    stc_gpio_init_t stcGpio;
    stc_spi_init_t stcSpi;
    stc_dma_init_t stcDma;
    stc_irq_signin_config_t stcIrq;

    s_xfer_done = xSemaphoreCreateBinary();

    /* 1. GPIO 初始化 */
    (void)GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDrv = PIN_HIGH_DRV;
    (void)GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, &stcGpio);
    (void)GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, &stcGpio);
    (void)GPIO_Init(SPI_SCK_PORT,  SPI_SCK_PIN,  &stcGpio);
    GPIO_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_FUNC_40);
    GPIO_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, GPIO_FUNC_41);
    GPIO_SetFunc(SPI_SCK_PORT,  SPI_SCK_PIN,  GPIO_FUNC_43);

    /* CS 配置为推挽输出，默认高电平 */
    stcGpio.u16PinDir = PIN_DIR_OUT;
    (void)GPIO_Init(SPI_CS_PORT, SPI_CS_PIN, &stcGpio);
    spi_cs_high();

    /* 2. SPI 初始化：主机，全双工，MODE0，CLK/8，MSB first */
    FCG_Fcg1PeriphClockCmd(SPI_CLK, ENABLE);
    SPI_StructInit(&stcSpi);
    stcSpi.u32WireMode          = SPI_4_WIRE;
    stcSpi.u32TransMode         = SPI_FULL_DUPLEX;
    stcSpi.u32MasterSlave       = SPI_MASTER;
    stcSpi.u32SpiMode           = SPI_MD_0;
    stcSpi.u32BaudRatePrescaler = SPI_BR_CLK_DIV8;
    stcSpi.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpi.u32FirstBit          = SPI_FIRST_MSB;
    (void)SPI_Init(SPI_UNIT, &stcSpi);

    /* 3. DMA 初始化 */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS, ENABLE);
    (void)DMA_StructInit(&stcDma);
    stcDma.u32BlockSize  = 1UL;
    stcDma.u32DataWidth  = DMA_DATAWIDTH_8BIT;
    stcDma.u32TransCount = 1UL;

    /* TX 通道：源地址递增，目标固定（SPI DR） */
    stcDma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDma.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    stcDma.u32SrcAddr     = (uint32_t)&s_dummy;
    stcDma.u32DestAddr    = (uint32_t)(&SPI_UNIT->DR);
    (void)DMA_Init(DMA_UNIT, DMA_TX_CH, &stcDma);
    AOS_SetTriggerEventSrc(DMA_TX_TRIG, SPI_TX_EVT);

    /* RX 通道：源固定（SPI DR），目标地址递增 */
    stcDma.u32IntEn       = DMA_INT_ENABLE;
    stcDma.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;
    stcDma.u32DestAddrInc = DMA_DEST_ADDR_INC;
    stcDma.u32SrcAddr     = (uint32_t)(&SPI_UNIT->DR);
    stcDma.u32DestAddr    = (uint32_t)&s_dummy;
    (void)DMA_Init(DMA_UNIT, DMA_RX_CH, &stcDma);
    AOS_SetTriggerEventSrc(DMA_RX_TRIG, SPI_RX_EVT);

    /* 4. RX 完成中断 */
    stcIrq.enIntSrc    = DMA_RX_INT_SRC;
    stcIrq.enIRQn      = DMA_RX_IRQn;
    stcIrq.pfnCallback = dma_rx_irq_handler;
    (void)INTC_IrqSignIn(&stcIrq);
    NVIC_ClearPendingIRQ(DMA_RX_IRQn);
    NVIC_SetPriority(DMA_RX_IRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(DMA_RX_IRQn);

    DMA_Cmd(DMA_UNIT, ENABLE);
}

/**
 * @brief  向从机写数据
 * @param  reg    寄存器地址（1 字节，先发）
 * @param  buf    数据缓冲区
 * @param  len    字节数
 * @retval LL_OK
 */
int32_t spi_write(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    spi_cs_low();
    /* 先发寄存器地址 */
    spi_dma_transfer(&reg, NULL, 1UL);
    /* 再发数据 */
    spi_dma_transfer(buf, NULL, len);
    spi_cs_high();
    return LL_OK;
}

/**
 * @brief  从从机读数据
 * @param  reg    寄存器地址（1 字节，先发）
 * @param  buf    接收缓冲区
 * @param  len    字节数
 * @retval LL_OK
 */
int32_t spi_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
    spi_cs_low();
    /* 先发寄存器地址 */
    spi_dma_transfer(&reg, NULL, 1UL);
    /* 再读数据 */
    spi_dma_transfer(NULL, buf, len);
    spi_cs_high();
    return LL_OK;
}
