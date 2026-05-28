#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include "hc32_ll.h"

#define RGB_LED_CHANNEL_COUNT           (4U)
#define RGB_LED_DEFAULT_LED_COUNT       (9U)

void rgb_led_init(void);
void rgb_led_clear(uint8_t ch);
void rgb_led_write_pixel(uint8_t ch, uint16_t index, uint32_t color);
void rgb_led_refresh(uint8_t ch);

#endif
