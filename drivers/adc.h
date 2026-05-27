#ifndef DRV_ADC_H__
#define DRV_ADC_H__

#include "hc32_ll.h"
#include "hc32_ll_aos.h"
#include "hc32_ll_adc.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_utility.h"

typedef struct {
    uint16_t port;          // GPIO 端口, 如: GPIO_PORT_A
    uint16_t pin;           // GPIO 脚位, 如: GPIO_PIN_06
    uint16_t ch;            // ADC 通道, 如: ADC_CH6
    void (*cb)(float val);  // 值回调接口
} adc_obj_t;


void adc_init(adc_obj_t* obj, uint8_t count);


#endif /* DRV_ADC_H__ */
