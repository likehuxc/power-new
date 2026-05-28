#ifndef LIGHT_CTRL_H__
#define LIGHT_CTRL_H__

#include <stdint.h>

typedef enum {
    LIGHT_MODE_OFF = 0,
    LIGHT_MODE_ON = 1,
    LIGHT_MODE_BLINK_THEN_OFF = 2,
    LIGHT_MODE_BREATH = 3,
    LIGHT_MODE_BLINK_THEN_ON = 4,
    LIGHT_MODE_RGB = 5,
    LIGHT_MODE_MARQUEE_R2L = 8,
    LIGHT_MODE_RUNNING_RIGHT = 9,
    LIGHT_MODE_STARTUP = 10,
    LIGHT_MODE_SHUTDOWN = 11,
} LIGHT_MODE_E;

typedef struct {
    uint16_t light_number;
    uint8_t count;
    LIGHT_MODE_E mode;
    uint16_t cycle_ms;
    uint32_t colcor;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} light_control_param_t;

void light_ctrl_init(void);
void LightCtrl_SetParam(const light_control_param_t *param);

#endif /* LIGHT_CTRL_H__ */
