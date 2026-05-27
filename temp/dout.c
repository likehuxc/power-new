#include "dout.h"

void dout_init(const dout_obj_t* obj)
{
    stc_gpio_init_t g;

    if(obj) {
        GPIO_StructInit(&g);
        g.u16PinState = PIN_STAT_RST;
        g.u16PinDir = PIN_DIR_OUT;
        GPIO_Init(obj->port, obj->pin, &g);
    }
}

void dout_set_active(const dout_obj_t* obj)
{
    if(obj) {
        if(obj->active_level) {
            GPIO_SetPins(obj->port, obj->pin);
        } else {
            GPIO_ResetPins(obj->port, obj->pin);
        }
    }
}

void dout_set_inactive(const dout_obj_t* obj)
{
    if(obj) {
        if((!obj->active_level)) {
            GPIO_SetPins(obj->port, obj->pin);
        } else {
            GPIO_ResetPins(obj->port, obj->pin);
        }
    }
}

void dout_set(const dout_obj_t* obj, uint8_t active)
{
    if(active) dout_set_active(obj); else dout_set_inactive(obj);
}



