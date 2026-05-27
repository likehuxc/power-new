#ifndef __DIN_H__
#define __DIN_H__

#include "hc32_ll.h"

/* 数字输入 IO 回调类型 */
typedef void(*din_callback_t)(uint8_t event);

/* 数字输入 IO 类型 */
typedef struct {
    const char* name;
    uint8_t  port;
    uint16_t pin;
    uint8_t  jitter;
    uint8_t  active_level;
    din_callback_t cb;
    uint8_t  state;
    uint8_t  ticks;
    uint8_t  active;
} din_obj_t;

/* 定义输入脚 */
#define def_din_pin(_name, _port, _pin, _jitter, _active_level)     \
    din_obj_t _name = {     \
        .name = #_name,     \
        .port = (_port),    \
        .pin = (_pin),      \
        .jitter = (_jitter),                \
        .active_level = (_active_level),    \
    }

/* 声明输入脚 */
#define declare_din_pin(_name) \
    extern din_obj_t _name

void din_init(din_obj_t* obj);
void din_set_callback(din_obj_t* obj, din_callback_t cb);
int  is_din_active(din_obj_t* obj);
void din_handler(void);

#endif
