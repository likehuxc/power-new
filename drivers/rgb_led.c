#include "rgb_led.h"

#define RGB_LED_BITS_PER_PIXEL          (24U)
#define RGB_LED_STOP_CNT                (1U)
#define RGB_LED_BUF_COUNT               ((RGB_LED_DEFAULT_LED_COUNT * RGB_LED_BITS_PER_PIXEL) + RGB_LED_STOP_CNT)

#define WS2812_TH_US                    (0.8f)
#define WS2812_TL_US                    (0.45f)

#define RGB888_TO_GRB888(rgb) \
    ((((rgb) & 0xFF0000UL) >> 8) | (((rgb) & 0x00FF00UL) << 8) | ((rgb) & 0x0000FFUL))

typedef struct {
    uint16_t port;
    uint16_t pin;
    uint8_t func;
    CM_TMRA_TypeDef *tmra;
    uint32_t tmra_ch;
    CM_DMA_TypeDef *dma;
    uint32_t dma_ch;
    uint32_t dma_tc_flag;
    uint32_t dma_ch_en;
    uint32_t aos_dst;
    en_event_src_t aos_src;
    __IO uint16_t *cmp;
    uint16_t *buf;
} rgb_led_hw_t;     /* 硬件映射 */

/* 每路灯带一份 DMA 发送缓冲：24bit/灯 + 末尾复位低电平占位。 */
static __ALIGNED(4) uint16_t s_ch1_buf[RGB_LED_BUF_COUNT];
static __ALIGNED(4) uint16_t s_ch2_buf[RGB_LED_BUF_COUNT];
static __ALIGNED(4) uint16_t s_ch3_buf[RGB_LED_BUF_COUNT];
static __ALIGNED(4) uint16_t s_ch4_buf[RGB_LED_BUF_COUNT];

/*
 * CT02 灯带硬件映射。
 * 原理图对应关系：
 *   PD12 -> TIMA4_PWM1
 *   PD13 -> TIMA4_PWM2
 *   PD14 -> TIMA4_PWM3
 *   PD15 -> TIMA4_PWM4
 * 这里使用 TIMA4 溢出事件触发 DMA，把下一位的比较值搬到对应 CMPARx。
 */
static const rgb_led_hw_t s_rgb_led_hw[RGB_LED_CHANNEL_COUNT] = {
    { GPIO_PORT_D, GPIO_PIN_12, GPIO_FUNC_4, CM_TMRA_4, TMRA_CH1,
      CM_DMA2, DMA_CH0, DMA_FLAG_TC_CH0, DMA_CHEN_CHEN_0,
      AOS_DMA2_0, EVT_SRC_TMRA_4_OVF, &CM_TMRA_4->CMPAR1, s_ch1_buf },
    { GPIO_PORT_D, GPIO_PIN_13, GPIO_FUNC_4, CM_TMRA_4, TMRA_CH2,
      CM_DMA2, DMA_CH1, DMA_FLAG_TC_CH1, DMA_CHEN_CHEN_1,
      AOS_DMA2_1, EVT_SRC_TMRA_4_OVF, &CM_TMRA_4->CMPAR2, s_ch2_buf },
    { GPIO_PORT_D, GPIO_PIN_14, GPIO_FUNC_4, CM_TMRA_4, TMRA_CH3,
      CM_DMA2, DMA_CH2, DMA_FLAG_TC_CH2, DMA_CHEN_CHEN_2,
      AOS_DMA2_2, EVT_SRC_TMRA_4_OVF, &CM_TMRA_4->CMPAR3, s_ch3_buf },
    { GPIO_PORT_D, GPIO_PIN_15, GPIO_FUNC_4, CM_TMRA_4, TMRA_CH4,
      CM_DMA2, DMA_CH3, DMA_FLAG_TC_CH3, DMA_CHEN_CHEN_3,
      AOS_DMA2_3, EVT_SRC_TMRA_4_OVF, &CM_TMRA_4->CMPAR4, s_ch4_buf },
};

static uint16_t s_t0h_cmp = 0U;
static uint16_t s_t1h_cmp = 0U;

static void rgb_led_fill(uint16_t *buf, uint16_t value)
{
    uint16_t i;

    for (i = 0U; i < RGB_LED_BUF_COUNT; i++) {
        buf[i] = value;
    }
}

void rgb_led_init(void)
{
    stc_tmra_init_t tmr;
    stc_tmra_pwm_init_t pwm;
    stc_dma_init_t dma;
    stc_clock_freq_t freq;
    uint32_t timer_clk_mhz;
    uint8_t i;

    CLK_GetClockFreq(&freq);

    TMRA_StructInit(&tmr);
    tmr.sw_count.u8CountMode = TMRA_MD_SAWTOOTH;
    tmr.sw_count.u8CountDir  = TMRA_DIR_UP;
    tmr.sw_count.u8ClockDiv  = TMRA_CLK_DIV2;

    timer_clk_mhz = freq.u32Pclk1Freq / (1UL << (tmr.sw_count.u8ClockDiv >> TMRA_BCSTRL_CKDIV_POS));
    timer_clk_mhz /= 1000000UL;

    /* WS2812 单 bit 周期约 1.25us，TIMA 周期值按当前 PCLK1 自动换算。 */
    tmr.u32PeriodValue = (uint16_t)((WS2812_TH_US + WS2812_TL_US) * (float)timer_clk_mhz + 0.5f - 1.0f);

    /* PWM 极性配置为反相输出，因此 0/1 码的比较值在这里按旧驱动逻辑对调。 */
    s_t0h_cmp = (uint16_t)(WS2812_TH_US * (float)timer_clk_mhz + 0.5f - 1.0f);
    s_t1h_cmp = (uint16_t)(WS2812_TL_US * (float)timer_clk_mhz + 0.5f - 1.0f);

    TMRA_PWM_StructInit(&pwm);
    pwm.u16StartPolarity = TMRA_PWM_LOW;
    pwm.u16StopPolarity = TMRA_PWM_LOW;
    pwm.u16CompareMatchPolarity = TMRA_PWM_HIGH;
    pwm.u16PeriodMatchPolarity = TMRA_PWM_LOW;
    pwm.u32CompareValue = 0xFFFFUL;

    DMA_StructInit(&dma);
    dma.u32IntEn       = DMA_INT_DISABLE;
    dma.u32BlockSize   = 1UL;
    dma.u32DataWidth   = DMA_DATAWIDTH_16BIT;
    dma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    dma.u32DestAddrInc = DMA_DEST_ADDR_FIX;

    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_4, ENABLE);
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE);

    /* 四路共用 TIMA4 的同一周期，只分别打开 PWM1~PWM4 输出。 */
    TMRA_Init(CM_TMRA_4, &tmr);

    for (i = 0U; i < RGB_LED_CHANNEL_COUNT; i++) {
        const rgb_led_hw_t *hw = &s_rgb_led_hw[i];

        GPIO_SetFunc(hw->port, hw->pin, hw->func);
        GPIO_AnalogCmd(hw->port, hw->pin, DISABLE);

        TMRA_PWM_Init(hw->tmra, hw->tmra_ch, &pwm);
        TMRA_PWM_OutputCmd(hw->tmra, hw->tmra_ch, ENABLE);

        /* AOS 将 TIMA4 溢出事件路由到 DMA2_CHx，实现逐 bit 自动更新占空比。 */
        AOS_SetTriggerEventSrc(hw->aos_dst, hw->aos_src);
        DMA_Init(hw->dma, hw->dma_ch, &dma);
        DMA_Cmd(hw->dma, ENABLE);

        rgb_led_fill(hw->buf, 0xFFFFU);
    }

    TMRA_Start(CM_TMRA_4);
}

void rgb_led_clear(uint8_t ch)
{
    uint16_t i;

    if (ch >= RGB_LED_CHANNEL_COUNT) {
        return;
    }

    for (i = 0U; i < (RGB_LED_BUF_COUNT - RGB_LED_STOP_CNT); i++) {
        s_rgb_led_hw[ch].buf[i] = s_t0h_cmp;
    }
}

void rgb_led_write_pixel(uint8_t ch, uint16_t index, uint32_t color)
{
    uint16_t i;
    uint16_t *buf;

    if ((ch >= RGB_LED_CHANNEL_COUNT) || (index >= RGB_LED_DEFAULT_LED_COUNT)) {
        return;
    }

    color = RGB888_TO_GRB888(color);
    buf = s_rgb_led_hw[ch].buf;

    for (i = 0U; i < RGB_LED_BITS_PER_PIXEL; i++) {
        buf[(index * RGB_LED_BITS_PER_PIXEL) + i] = ((color & 0x800000UL) != 0UL) ? s_t1h_cmp : s_t0h_cmp;
        color <<= 1UL;
    }
}

void rgb_led_refresh(uint8_t ch)
{
    const rgb_led_hw_t *hw;

    if (ch >= RGB_LED_CHANNEL_COUNT) {
        return;
    }

    hw = &s_rgb_led_hw[ch];

    if ((hw->dma->CHEN & hw->dma_ch_en) != 0UL) {
        /* 避免改写正在发送的 DMA 通道，先等上一帧搬运和一次 PWM 周期结束。 */
        while (DMA_GetTransCompleteStatus(hw->dma, hw->dma_tc_flag) == RESET) {
        }
        TMRA_ClearStatus(hw->tmra, TMRA_FLAG_OVF);
        while (TMRA_GetStatus(hw->tmra, TMRA_FLAG_OVF) != SET) {
        }
    }

    /* 重新装载本路缓冲区到对应 CMPARx，下一次 TIMA4 溢出开始输出新帧。 */
    DMA_ClearTransCompleteStatus(hw->dma, hw->dma_tc_flag);
    DMA_ChCmd(hw->dma, hw->dma_ch, DISABLE);
    DMA_SetSrcAddr(hw->dma, hw->dma_ch, (uint32_t)&hw->buf[0]);
    DMA_SetDestAddr(hw->dma, hw->dma_ch, (uint32_t)hw->cmp);
    DMA_SetTransCount(hw->dma, hw->dma_ch, RGB_LED_BUF_COUNT);
    DMA_ChCmd(hw->dma, hw->dma_ch, ENABLE);
}
