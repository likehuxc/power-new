/**
 *******************************************************************************
 * @file  drv_led.c
 *******************************************************************************
 */

#include "drv_led.h"

#include "hc32_ll.h"

#define DRV_LED_PORT    (GPIO_PORT_B)
#define DRV_LED_PIN     (GPIO_PIN_07)

void DrvLed_Init(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState = PIN_STAT_SET;
    stcGpioInit.u16PinDir   = PIN_DIR_OUT;
    (void)GPIO_Init(DRV_LED_PORT, DRV_LED_PIN, &stcGpioInit);
}

void DrvLed_Toggle(void)
{
    GPIO_TogglePins(DRV_LED_PORT, DRV_LED_PIN);
}
