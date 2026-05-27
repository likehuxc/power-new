/**
 *******************************************************************************
 * @file  power_manage.c
 * @brief ADC 采样管理：DMA 中断 callback 更新全局量，任务侧直接读取
 *
 * 数据流：
 *   adc.c(DMA 中断) → 引脚电压(V) → callback → 分压还原 → 滑动滤波 → 全局变量
 *
 * 通道与扫描顺序见 s_adc_table，与原理图 ADC1 配置一致。
 *******************************************************************************
 */

#include "power_manage.h"

#include "adc.h"
#include "saf_filter.h"

/* 分压还原系数：ADC 引脚电压 × 系数 = 实际电压(V)，与 d5w_pmu 一致 */
#define ADC_SCALE_FACTOR        (22.27f)
/* 变化小于该阈值时不更新全局变量，减少任务侧无效抖动 */
#define ADC_UPDATE_THRESHOLD    (0.1f)
/* 软件滑动平均窗口长度（硬件已做 256 次平均） */
#define SAF_BUF_SIZE            (10U)

#define __abs(a, b)    (((a) > (b)) ? ((a) - (b)) : ((b) - (a)))

/* -----------------------------------------------------------------------
 * 滤波后的监测量，供电源管理/电池等任务读取
 * ----------------------------------------------------------------------- */
float g_motor_bus_voltage  = 0.0f;  /* 电机母线电压 AI_MOTOR_BUS   */
float g_chg_adpt_voltage   = 0.0f;  /* 线充电压     AI_CHG_ADPT    */
float g_ibat_out_2         = 0.0f;  /* 电池2电流    IBAT_OUT_2     */
float g_chg_pile_voltage   = 0.0f;  /* 桩充电压     AI_CHG_PILE    */
float g_vbat_in_1_voltage  = 0.0f;  /* 电池1电压    AI_VBAT_IN_1   */
float g_vbat_in_2_voltage  = 0.0f;  /* 电池2电压    AI_VBAT_IN_2   */
float g_ibat_out_1         = 0.0f;  /* 电池1电流    IBAT_OUT_1     */
float g_vbat_in_voltage    = 0.0f;  /* 总充电口电压 AI_VBAT_IN     */

static float s_saf_buf[8][SAF_BUF_SIZE];
static saf_t s_saf_obj[8];

/* 以下 callback 在 DMA 中断上下文执行，仅做轻量运算，禁止阻塞/打印 */

static void on_ai_motor_bus_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[0], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev                  = val;
        g_motor_bus_voltage   = val;
    }
}

static void on_ai_chg_adpt_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[1], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev                 = val;
        g_chg_adpt_voltage   = val;
    }
}

static void on_ibat_out_2_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[2], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev           = val;
        g_ibat_out_2   = val;
    }
}

static void on_ai_chg_pile_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[3], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev                 = val;
        g_chg_pile_voltage   = val;
    }
}

static void on_ai_vbat_in_1_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[4], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev                  = val;
        g_vbat_in_1_voltage   = val;
    }
}

static void on_ai_vbat_in_2_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[5], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev                  = val;
        g_vbat_in_2_voltage   = val;
    }
}

static void on_ibat_out_1_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[6], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev           = val;
        g_ibat_out_1   = val;
    }
}

static void on_ai_vbat_in_update(float val)
{
    static float prev = 0.0f;

    val *= ADC_SCALE_FACTOR;
    val = sliding_average_filter(&s_saf_obj[7], val);
    if (__abs(prev, val) > ADC_UPDATE_THRESHOLD) {
        prev                = val;
        g_vbat_in_voltage   = val;
    }
}

/* ADC 通道表：顺序即硬件扫描顺序 */
static adc_obj_t s_adc_table[] = {
    { GPIO_PORT_A, GPIO_PIN_01, ADC_CH1,  on_ai_motor_bus_update  }, /* 1 AI_MOTOR_BUS  电机电压   */
    { GPIO_PORT_A, GPIO_PIN_06, ADC_CH6,  on_ai_chg_adpt_update   }, /* 2 AI_CHG_ADPT   线充电压   */
    { GPIO_PORT_B, GPIO_PIN_00, ADC_CH8,  on_ibat_out_2_update    }, /* 3 IBAT_OUT_2    电池2电流  */
    { GPIO_PORT_B, GPIO_PIN_01, ADC_CH9,  on_ai_chg_pile_update   }, /* 4 AI_CHG_PILE   桩充电压   */
    { GPIO_PORT_C, GPIO_PIN_02, ADC_CH12, on_ai_vbat_in_1_update  }, /* 5 AI_VBAT_IN_1  电池1电压  */
    { GPIO_PORT_C, GPIO_PIN_03, ADC_CH13, on_ai_vbat_in_2_update  }, /* 6 AI_VBAT_IN_2  电池2电压  */
    { GPIO_PORT_C, GPIO_PIN_04, ADC_CH14, on_ibat_out_1_update    }, /* 7 IBAT_OUT_1    电池1电流  */
    { GPIO_PORT_C, GPIO_PIN_05, ADC_CH15, on_ai_vbat_in_update    }, /* 8 AI_VBAT_IN    总充电口电压 */
};

void PowerManage_Init(void)
{
    uint32_t i;

    /* 初始化各路软件滑动滤波器 */
    for (i = 0U; i < (sizeof(s_saf_obj) / sizeof(s_saf_obj[0])); i++) {
        sliding_average_filter_init(&s_saf_obj[i], s_saf_buf[i], SAF_BUF_SIZE);
    }

    /* 启动 ADC1 DMA 连续扫描，结果通过 callback 更新全局变量 */
    adc_init(s_adc_table, (uint8_t)(sizeof(s_adc_table) / sizeof(s_adc_table[0])));
}

void PowerManage_Task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        LOG_INFO("motor_bus: %.2fV, chg_adpt: %.2fV, ibat_out_2: %.2fA, chg_pile: %.2fV, vbat_in_1: %.2fV, vbat_in_2: %.2fV, ibat_out_1: %.2fA, vbat_in: %.2fV",
                 g_motor_bus_voltage, g_chg_adpt_voltage, g_ibat_out_2, g_chg_pile_voltage, g_vbat_in_1_voltage, g_vbat_in_2_voltage, g_ibat_out_1, g_vbat_in_voltage);
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
