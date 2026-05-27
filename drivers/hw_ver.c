/**
 *******************************************************************************
 * @file  hw_ver.c
 * @brief 硬件版本识别：PB5(Bit0) + PB6(Bit1) 上拉输入读取
 *
 * 使用方式：
 *   系统启动时调用 HwVer_Init() 初始化引脚，之后通过 HwVer_Get() 读取版本号。
 *   版本号在 PCB 生产时通过焊接电阻决定，运行期间不会变化。
 *******************************************************************************
 */

#include "hw_ver.h"
#include "hc32_ll.h"
#include "hc32_ll_gpio.h"

/* 初始化硬件版本引脚 */
void hw_version_init(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_IN;
    stcGpioInit.u16PullUp = PIN_PU_ON;

    (void)GPIO_Init(HW_VERSION_0_PORT, HW_VERSION_0_PIN, &stcGpioInit);
    (void)GPIO_Init(HW_VERSION_1_PORT, HW_VERSION_1_PIN, &stcGpioInit);
}

/* 读取硬件版本 */
uint8_t hw_version_get(void)
{
    uint8_t ver = 0U;

    if (GPIO_ReadInputPins(HW_VERSION_0_PORT, HW_VERSION_0_PIN) != PIN_RESET) {
        ver |= 0x01U;
    }
    if (GPIO_ReadInputPins(HW_VERSION_1_PORT, HW_VERSION_1_PIN) != PIN_RESET) {
        ver |= 0x02U;
    }
    return ver;
}
