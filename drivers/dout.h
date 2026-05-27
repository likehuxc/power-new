#ifndef __DOUT_H__
#define __DOUT_H__

#include "hc32_ll.h"

typedef struct {
    uint16_t port;
    uint16_t pin;
    uint8_t active_level;
} dout_obj_t;

/* 定义输出脚 */
#define def_dout_pin(_name, _port, _pin, _active_level) \
    const dout_obj_t _name = { .port = (_port), .pin = (_pin), .active_level = (_active_level) }

/* 声明输出脚 */
#define declare_dout_pin(_name) \
    extern const dout_obj_t _name

void dout_init(const dout_obj_t* obj);
void dout_set_active(const dout_obj_t* obj);
void dout_set_inactive(const dout_obj_t* obj);
void dout_set(const dout_obj_t* obj, uint8_t active);

#endif
