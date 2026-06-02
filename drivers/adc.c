#include "adc.h"

// ADC 转换结果
// ADC 总共 17 通道, 为了简化 DMA 地址不连续传输配置, 改为直接传输 17 通道结果
#define ADC_CH_COUNT            (17)

/* ADC DMA uses DMA1_CH2. DMA1_CH0/CH1 are reserved for USART1 RX/TX. */
#define ADC_DMA_UNIT            (CM_DMA1)
#define ADC_DMA_CH              (DMA_CH2)
#define ADC_DMA_FCG_ENABLE()    (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define ADC_DMA_TRIG_SEL        (AOS_DMA1_2)
#define ADC_DMA_BTC_INT         (DMA_INT_BTC_CH2)
#define ADC_DMA_BTC_FLAG        (DMA_FLAG_BTC_CH2)
#define ADC_DMA_BTC_INT_SRC     (INT_SRC_DMA1_BTC2)

static __ALIGNED(4) uint16_t ADC_BUFF[ADC_CH_COUNT];

static adc_obj_t* adc_obj = 0;
static uint8_t    adc_obj_cnt = 0;

static void DMA_IrqCallback(void)
{
    float val;
    uint8_t i;

    DMA_ClearTransCompleteStatus(ADC_DMA_UNIT, ADC_DMA_BTC_FLAG);
    ADC_Start(CM_ADC1);

    for(i = 0; i < adc_obj_cnt; i ++) {
        if((adc_obj[i].ch <= 16) && (adc_obj[i].cb)) {
            val = ADC_BUFF[adc_obj[i].ch];
            val *= 3.3f; val /= 4096;
            adc_obj[i].cb(val);
        }
    }
}

void adc_init(adc_obj_t* obj, uint8_t count)
{
    stc_adc_init_t  adc;
    stc_gpio_init_t gpio;
    stc_dma_init_t  dma;
    stc_dma_repeat_init_t   dma_rpt;
    stc_irq_signin_config_t irq;
    uint8_t i;

    if((!obj) || (count <= 0)) return;
    
    adc_obj = obj;
    adc_obj_cnt = count;

    // GPIO 配置
    GPIO_StructInit(&gpio);
    gpio.u16PinAttr = PIN_ATTR_ANALOG;
    for(i = 0; i < count; i ++) {
        GPIO_Init(obj[i].port, obj[i].pin, &gpio);
    }
    // 使能 ADC 时钟
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);
    // ADC 初始化
    ADC_StructInit(&adc);
    ADC_Init(CM_ADC1, &adc);
    // ADC 通道使能
    for(i = 0; i < count; i ++) {
        ADC_ChCmd(CM_ADC1, ADC_SEQ_A, obj[i].ch, ENABLE);
    }

    // 使能 DMA 时钟
    ADC_DMA_FCG_ENABLE();
    // DMA 初始化
    DMA_StructInit(&dma);
    dma.u32IntEn       = DMA_INT_ENABLE;
    dma.u32SrcAddr     = ((uint32_t)&CM_ADC1->DR0);
    dma.u32DestAddr    = ((uint32_t)(&ADC_BUFF[0U]));
    dma.u32DataWidth   = DMA_DATAWIDTH_16BIT;
    dma.u32BlockSize   = ADC_CH_COUNT;
    dma.u32TransCount  = 0; /* 0 表示无限次传输 */
    dma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    dma.u32DestAddrInc = DMA_DEST_ADDR_INC;
    DMA_Init(ADC_DMA_UNIT, ADC_DMA_CH, &dma);
    // DMA 重复搬运配置
    dma_rpt.u32Mode      = DMA_RPT_BOTH;
    dma_rpt.u32SrcCount  = ADC_CH_COUNT;
    dma_rpt.u32DestCount = ADC_CH_COUNT;
    DMA_RepeatInit(ADC_DMA_UNIT, ADC_DMA_CH, &dma_rpt);

    // 使能 AOS 时钟
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
    // 设置 DMA 触发源
    AOS_SetTriggerEventSrc(ADC_DMA_TRIG_SEL, EVT_SRC_ADC1_EOCA);

    // DMA 中断配置
#define DMA_INT_TYPE    (ADC_DMA_BTC_INT)
#define DMA_INT_SRC     (ADC_DMA_BTC_INT_SRC)
#define DMA_INT_IRQn    (INT038_IRQn)
#define DMA_INT_PRIO    (DDL_IRQ_PRIO_03)

    irq.enIntSrc    = DMA_INT_SRC;
    irq.enIRQn      = DMA_INT_IRQn;
    irq.pfnCallback = &DMA_IrqCallback;
    INTC_IrqSignIn(&irq);
    DMA_ClearTransCompleteStatus(ADC_DMA_UNIT, ADC_DMA_BTC_FLAG);

    // NVIC 配置
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_SetPriority(DMA_INT_IRQn, DMA_INT_PRIO);
    NVIC_EnableIRQ(DMA_INT_IRQn);

    // 使能 DMA
    DMA_Cmd(ADC_DMA_UNIT, ENABLE);
    DMA_ChCmd(ADC_DMA_UNIT, ADC_DMA_CH, ENABLE);
    // 硬件平均值设置
    ADC_ConvDataAverageConfig(CM_ADC1, ADC_AVG_CNT256);
    for(i = 0; i < count; i ++) {
        ADC_ConvDataAverageChCmd(CM_ADC1, obj[i].ch, ENABLE);
    }

    ADC_Start(CM_ADC1);
}
