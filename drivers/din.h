#ifndef __DIN_H__
#define __DIN_H__

#include "hc32_ll.h"

/* 数字输入 IO 回调类型 */
typedef void(*din_callback_t)(uint8_t event);

/* 数字输入 IO 类型 */
typedef struct {
    const char* name;       /* 输入脚名称 */
    uint8_t  port;          /* 输入脚端口 */
    uint16_t pin;           /* 输入脚引脚 */
    uint8_t  jitter;        /* 输入脚抖动阈值 */
    uint8_t  active_level;  /* 输入脚有效电平 */
    din_callback_t cb;      /* 输入脚回调函数 */
    uint8_t  state;         /* 输入脚状态 */
    uint8_t  ticks;         /* 输入脚抖动计数 */
    uint8_t  active;        /* 输入脚是否激活 */
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
