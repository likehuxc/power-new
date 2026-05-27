#include "din.h"
#include "fault_report.h"

// 使能调试
#define DIN_DEBUG       (1)
// 数字输入 IO 数量
#define IO_MAX_NUM      (64)


static din_obj_t* dio[IO_MAX_NUM] = {0};

// 数字输入 IO 端口初始化
void din_init(din_obj_t* obj)
{
    stc_gpio_init_t g;
    uint32_t i;

    obj->cb = 0;
    obj->state = 0;
    obj->ticks = 0;
    obj->active = 0;

    // IO 初始化
    GPIO_StructInit(&g);
    g.u16PullUp = PIN_PU_ON;
    g.u16PinDir = PIN_DIR_IN;
    GPIO_Init(obj->port, obj->pin, &g);

    __disable_irq();

    for(i = 0; i < IO_MAX_NUM; i ++) {
        if(!dio[i]) { 
            dio[i] = obj; DEBUG_PRINT("din [%s] init OK\r\n", obj->name); break;
        }
    }
    if(i == IO_MAX_NUM) {
        DEBUG_PRINT("din [%s] init ERROR\r\n", obj->name);
    }
    __enable_irq();
}

// 设置 IO 有效状态变化回调
void din_set_callback(din_obj_t* obj, din_callback_t cb)
{
    if(obj) obj->cb = cb;
}

// 检测 IO 当前电平是否有效
int is_din_active(din_obj_t* obj)
{
    if(obj) {
        return obj->active;
    }
    return -1;
}

// IO 扫描处理
static void __scan_handler(din_obj_t* obj)
{
    uint8_t level;

    if(!obj) return; 

    level = GPIO_ReadInputPins(obj->port, obj->pin);
    
    switch(obj->state) {
        case 0: {
            if(level == (obj->active_level)) {
                if((++ obj->ticks) >= (obj->jitter)) {
                    if(obj->cb) obj->cb(1);
                    DEBUG_PRINT("din [%s] active\r\n", obj->name);
                    obj->active = 1;
                    obj->ticks = 0;
                    obj->state ++;
                }
            } else {
                obj->ticks = 0;
            }
        } break;
        case 1: {
            if(level != (obj->active_level)) {
                if((++ obj->ticks) >= (obj->jitter)) {
                    if(obj->cb) obj->cb(0);
                    DEBUG_PRINT("din [%s] inactive\r\n", obj->name);
                    obj->active = 0;
                    obj->state = 0;
                    obj->ticks = 0;
                }
            } else {
                obj->ticks = 0;
            }
        } break;

        default: break;
    }
}

// 数字输入 IO 处理
void din_handler(void)
{
    uint32_t i;

    if(perfc_is_time_out_ms(1)) {
        for(i = 0; i < IO_MAX_NUM; i ++) {
            if(dio[i]) __scan_handler(dio[i]);
        }
    }
}




