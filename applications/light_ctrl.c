#include "light_ctrl.h"

#include <math.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "log.h"
#include "rgb_led.h"

#define LIGHT_TASK_PERIOD_MS            (20U)
#define LIGHT_NUMBER_BASE               (200U)
#define LIGHT_DEFAULT_CYCLE_MS          (2000U)
#define LIGHT_BLUE                      (0x0000FFUL)

typedef struct {
    uint16_t step;
    uint8_t blink_done;
    LIGHT_MODE_E last_mode;
} light_anim_state_t;

typedef struct {
    uint8_t led_count;
    uint8_t pwm_channel;
    light_control_param_t param;
} light_param_t;

static uint32_t hsv_to_rgb(float h, float s, float v);
static void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, float *h, float *s, float *v);
static uint32_t light_param_color(const light_control_param_t *param, uint32_t fallback);
static uint16_t light_cycle_or_default(uint16_t cycle_ms);
static void light_write_all(uint8_t pwm_channel, uint8_t led_count, uint32_t color);
static void light_blink_n_then_off(light_anim_state_t *st, uint8_t blink_times, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count);
static void light_blink_n_then_on(light_anim_state_t *st, uint8_t blink_times, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count);
static void light_breathing(light_anim_state_t *st, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count);
static void light_marquee_r2l(light_anim_state_t *st, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count);
static void light_running_right(light_anim_state_t *st, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count);
static void light_startup(light_anim_state_t *st, uint32_t color, uint8_t pwm_channel, uint8_t led_count);
static void light_task(void *param);

static SemaphoreHandle_t s_light_mutex;

/*
 * 4 路灯带的上层配置。
 * light_number 预留给通信协议使用，默认 200~203 分别映射到底层 PWM 通道 0~3。
 * led_count 如后续 CT02 不同灯带长度不一致，只需要改这里。
 */
static light_param_t s_light_param[RGB_LED_CHANNEL_COUNT] = {
    {
        .led_count = RGB_LED_DEFAULT_LED_COUNT,
        .pwm_channel = 0U,
        .param = { .light_number = LIGHT_NUMBER_BASE + 0U, .mode = LIGHT_MODE_STARTUP, .colcor = LIGHT_BLUE },
    },
    {
        .led_count = RGB_LED_DEFAULT_LED_COUNT,
        .pwm_channel = 1U,
        .param = { .light_number = LIGHT_NUMBER_BASE + 1U, .mode = LIGHT_MODE_OFF, .colcor = LIGHT_BLUE },
    },
    {
        .led_count = RGB_LED_DEFAULT_LED_COUNT,
        .pwm_channel = 2U,
        .param = { .light_number = LIGHT_NUMBER_BASE + 2U, .mode = LIGHT_MODE_STARTUP, .colcor = LIGHT_BLUE },
    },
    {
        .led_count = RGB_LED_DEFAULT_LED_COUNT,
        .pwm_channel = 3U,
        .param = { .light_number = LIGHT_NUMBER_BASE + 3U, .mode = LIGHT_MODE_OFF, .colcor = LIGHT_BLUE },
    },
};

static light_anim_state_t s_anim_state[RGB_LED_CHANNEL_COUNT] = {
    { .last_mode = LIGHT_MODE_STARTUP },
    { .last_mode = LIGHT_MODE_OFF },
    { .last_mode = LIGHT_MODE_STARTUP },
    { .last_mode = LIGHT_MODE_OFF },
};

/* 协议层解析到新的灯控参数后调用此函数；灯效任务会在 20ms 周期内取走快照。 */
void LightCtrl_SetParam(const light_control_param_t *param)
{
    uint8_t i;

    if ((param == NULL) || (s_light_mutex == NULL)) {
        return;
    }

    xSemaphoreTake(s_light_mutex, portMAX_DELAY);
    for (i = 0U; i < RGB_LED_CHANNEL_COUNT; i++) {
        if (s_light_param[i].param.light_number == param->light_number) {
            s_light_param[i].param = *param;
            break;
        }
    }
    xSemaphoreGive(s_light_mutex);

    LOG_INFO("Light num:%u mode:%u cycle:%u color:0x%06lx",
             param->light_number,
             (uint8_t)param->mode,
             param->cycle_ms,
             param->colcor);
}

void light_ctrl_init(void)
{
    s_light_mutex = xSemaphoreCreateMutex();
    if (s_light_mutex == NULL) {
        LOG_ERROR("light mutex create failed");
        return;
    }

    (void)xTaskCreate(light_task, "light", 512, NULL, configMAX_PRIORITIES - 10, NULL);
}

static void light_task(void *param)
{
    uint8_t i;

    (void)param;

    rgb_led_init();

    for (i = 0U; i < RGB_LED_CHANNEL_COUNT; i++) {
        rgb_led_clear(i);
        rgb_led_refresh(i);
    }

    for (;;) {
        for (i = 0U; i < RGB_LED_CHANNEL_COUNT; i++) {
            light_param_t config;
            light_anim_state_t *anim = &s_anim_state[i];
            uint32_t color;
            uint16_t cycle_ms;

            xSemaphoreTake(s_light_mutex, portMAX_DELAY);
            config = s_light_param[i];
            xSemaphoreGive(s_light_mutex);

            /* 切换模式时清掉旧模式的步进计数，避免闪烁/跑马灯残留到新模式。 */
            if (config.param.mode != anim->last_mode) {
                anim->step = 0U;
                anim->blink_done = 0U;
                anim->last_mode = config.param.mode;
            }

            color = light_param_color(&config.param, LIGHT_BLUE);
            cycle_ms = light_cycle_or_default(config.param.cycle_ms);

            switch (config.param.mode) {
                case LIGHT_MODE_OFF:
                    light_write_all(config.pwm_channel, config.led_count, 0x000000UL);
                    break;

                case LIGHT_MODE_ON:
                    light_write_all(config.pwm_channel, config.led_count, color);
                    break;

                case LIGHT_MODE_BLINK_THEN_OFF:
                    light_blink_n_then_off(anim, config.param.count, cycle_ms, color, config.pwm_channel, config.led_count);
                    break;

                case LIGHT_MODE_BREATH:
                    light_breathing(anim, cycle_ms, color, config.pwm_channel, config.led_count);
                    break;

                case LIGHT_MODE_BLINK_THEN_ON:
                    light_blink_n_then_on(anim, config.param.count, cycle_ms, color, config.pwm_channel, config.led_count);
                    break;

                case LIGHT_MODE_RGB:
                    light_write_all(config.pwm_channel, config.led_count, color);
                    break;

                case LIGHT_MODE_MARQUEE_R2L:
                    light_marquee_r2l(anim, cycle_ms, color, config.pwm_channel, config.led_count);
                    break;

                case LIGHT_MODE_RUNNING_RIGHT:
                    light_running_right(anim, cycle_ms, color, config.pwm_channel, config.led_count);
                    break;

                case LIGHT_MODE_STARTUP:
                    light_startup(anim, color, config.pwm_channel, config.led_count);
                    break;

                case LIGHT_MODE_SHUTDOWN:
                default:
                    light_write_all(config.pwm_channel, config.led_count, 0x000000UL);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LIGHT_TASK_PERIOD_MS));
    }
}

static uint32_t light_param_color(const light_control_param_t *param, uint32_t fallback)
{
    /* 优先使用 24bit RGB 整色；兼容旧协议里分开的 r/g/b 字段。 */
    if (param->colcor != 0UL) {
        return param->colcor;
    }

    if ((param->r != 0U) || (param->g != 0U) || (param->b != 0U)) {
        return ((uint32_t)param->r << 16) | ((uint32_t)param->g << 8) | (uint32_t)param->b;
    }

    return fallback;
}

static uint16_t light_cycle_or_default(uint16_t cycle_ms)
{
    return (cycle_ms == 0U) ? LIGHT_DEFAULT_CYCLE_MS : cycle_ms;
}

static void light_write_all(uint8_t pwm_channel, uint8_t led_count, uint32_t color)
{
    uint16_t j;

    for (j = 0U; j < led_count; j++) {
        rgb_led_write_pixel(pwm_channel, j, color);
    }
    rgb_led_refresh(pwm_channel);
}

static void light_blink_n_then_off(light_anim_state_t *st, uint8_t blink_times, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count)
{
    uint16_t total_steps = (uint16_t)(cycle_ms / LIGHT_TASK_PERIOD_MS);
    uint16_t half_steps;

    if (total_steps == 0U) {
        total_steps = 1U;
    }
    half_steps = total_steps / 2U;

    if ((blink_times != 0U) && (st->blink_done >= blink_times)) {
        light_write_all(pwm_channel, led_count, 0x000000UL);
        return;
    }

    st->step++;
    if (st->step >= total_steps) {
        st->step = 0U;
        if (blink_times != 0U) {
            st->blink_done++;
        }
    }

    light_write_all(pwm_channel, led_count, (st->step < half_steps) ? color : 0x000000UL);
}

static void light_blink_n_then_on(light_anim_state_t *st, uint8_t blink_times, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count)
{
    uint16_t total_steps = (uint16_t)(cycle_ms / LIGHT_TASK_PERIOD_MS);
    uint16_t half_steps;

    if (total_steps == 0U) {
        total_steps = 1U;
    }
    half_steps = total_steps / 2U;

    if ((blink_times != 0U) && (st->blink_done >= blink_times)) {
        light_write_all(pwm_channel, led_count, color);
        return;
    }

    st->step++;
    if (st->step >= total_steps) {
        st->step = 0U;
        if (blink_times != 0U) {
            st->blink_done++;
        }
    }

    light_write_all(pwm_channel, led_count, (st->step < half_steps) ? color : 0x000000UL);
}

static void light_breathing(light_anim_state_t *st, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count)
{
    float h = 0.0f;
    float s = 0.0f;
    float v = 0.0f;
    uint16_t total_steps = (uint16_t)(cycle_ms / LIGHT_TASK_PERIOD_MS);
    uint16_t half_steps;
    float brightness;

    if (total_steps == 0U) {
        total_steps = 1U;
    }

    half_steps = total_steps / 2U;
    if (half_steps == 0U) {
        half_steps = 1U;
    }

    st->step = (uint16_t)((st->step + 1U) % total_steps);
    /* 亮度按半周期下降、半周期上升，色相/饱和度保持输入颜色不变。 */
    if (st->step < half_steps) {
        brightness = 1.0f - ((float)st->step / (float)half_steps);
    } else {
        brightness = (float)(st->step - half_steps) / (float)half_steps;
    }

    rgb_to_hsv((uint8_t)((color >> 16) & 0xFFU),
               (uint8_t)((color >> 8) & 0xFFU),
               (uint8_t)(color & 0xFFU),
               &h, &s, &v);
    (void)v;
    light_write_all(pwm_channel, led_count, hsv_to_rgb(h, s, brightness));
}

static void light_marquee_r2l(light_anim_state_t *st, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count)
{
    uint16_t total_steps = (uint16_t)(cycle_ms / LIGHT_TASK_PERIOD_MS);
    uint16_t steps_per_led;
    uint16_t led_idx;
    uint16_t j;

    if (led_count == 0U) {
        return;
    }

    if (total_steps == 0U) {
        total_steps = 1U;
    }

    st->step = (uint16_t)((st->step + 1U) % total_steps);
    steps_per_led = total_steps / led_count;
    if (steps_per_led == 0U) {
        steps_per_led = 1U;
    }

    led_idx = (uint16_t)(st->step / steps_per_led);
    if (led_idx >= led_count) {
        led_idx = (uint16_t)(led_count - 1U);
    }
    led_idx = (uint16_t)((led_count - 1U) - led_idx);

    for (j = 0U; j < led_count; j++) {
        rgb_led_write_pixel(pwm_channel, j, (j == led_idx) ? color : 0x000000UL);
    }
    rgb_led_refresh(pwm_channel);
}

static void light_running_right(light_anim_state_t *st, uint16_t cycle_ms, uint32_t color, uint8_t pwm_channel, uint8_t led_count)
{
    uint16_t total_steps = (uint16_t)(cycle_ms / LIGHT_TASK_PERIOD_MS);
    uint16_t steps_per_led;
    uint16_t head_idx;
    uint16_t j;

    if (led_count == 0U) {
        return;
    }

    if (total_steps == 0U) {
        total_steps = 1U;
    }

    st->step = (uint16_t)((st->step + 1U) % total_steps);
    steps_per_led = total_steps / led_count;
    if (steps_per_led == 0U) {
        steps_per_led = 1U;
    }

    head_idx = (uint16_t)(st->step / steps_per_led);
    if (head_idx >= led_count) {
        head_idx = (uint16_t)(led_count - 1U);
    }

    for (j = 0U; j < led_count; j++) {
        rgb_led_write_pixel(pwm_channel, j, (j <= head_idx) ? color : 0x000000UL);
    }
    rgb_led_refresh(pwm_channel);
}

static void light_startup(light_anim_state_t *st, uint32_t color, uint8_t pwm_channel, uint8_t led_count)
{
    float h = 0.0f;
    float s = 0.0f;
    float v = 0.0f;
    float t;

    if (st->step < 300U) {
        st->step++;
    }

    t = (float)st->step / 300.0f;
    /* smoothstep 渐亮曲线，开机时比线性变化更柔和。 */
    t = t * t * (3.0f - (2.0f * t));

    rgb_to_hsv((uint8_t)((color >> 16) & 0xFFU),
               (uint8_t)((color >> 8) & 0xFFU),
               (uint8_t)(color & 0xFFU),
               &h, &s, &v);
    light_write_all(pwm_channel, led_count, hsv_to_rgb(h, s, v * t));
}

static uint32_t hsv_to_rgb(float h, float s, float v)
{
    float c;
    float x;
    float m;
    float r1;
    float g1;
    float b1;
    uint8_t r;
    uint8_t g;
    uint8_t b;

    if (s <= 0.0f) {
        uint8_t val = (uint8_t)(v * 255.0f + 0.5f);
        return ((uint32_t)val << 16) | ((uint32_t)val << 8) | (uint32_t)val;
    }

    h = fmodf(h, 360.0f);
    if (h < 0.0f) {
        h += 360.0f;
    }

    c = v * s;
    x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    m = v - c;

    if (h < 60.0f) {
        r1 = c; g1 = x; b1 = 0.0f;
    } else if (h < 120.0f) {
        r1 = x; g1 = c; b1 = 0.0f;
    } else if (h < 180.0f) {
        r1 = 0.0f; g1 = c; b1 = x;
    } else if (h < 240.0f) {
        r1 = 0.0f; g1 = x; b1 = c;
    } else if (h < 300.0f) {
        r1 = x; g1 = 0.0f; b1 = c;
    } else {
        r1 = c; g1 = 0.0f; b1 = x;
    }

    r = (uint8_t)((r1 + m) * 255.0f + 0.5f);
    g = (uint8_t)((g1 + m) * 255.0f + 0.5f);
    b = (uint8_t)((b1 + m) * 255.0f + 0.5f);

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, float *h, float *s, float *v)
{
    float rf = (float)r / 255.0f;
    float gf = (float)g / 255.0f;
    float bf = (float)b / 255.0f;
    float cmax = fmaxf(rf, fmaxf(gf, bf));
    float cmin = fminf(rf, fminf(gf, bf));
    float delta = cmax - cmin;

    if (v != NULL) {
        *v = cmax;
    }

    if (s != NULL) {
        *s = (cmax <= 0.0f) ? 0.0f : (delta / cmax);
    }

    if (h == NULL) {
        return;
    }

    if (delta <= 0.0f) {
        *h = 0.0f;
    } else if (cmax == rf) {
        *h = 60.0f * fmodf((gf - bf) / delta, 6.0f);
    } else if (cmax == gf) {
        *h = 60.0f * (((bf - rf) / delta) + 2.0f);
    } else {
        *h = 60.0f * (((rf - gf) / delta) + 4.0f);
    }

    if (*h < 0.0f) {
        *h += 360.0f;
    }
}
