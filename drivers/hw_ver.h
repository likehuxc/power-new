/**
 *******************************************************************************
 * @file  hw_ver.h
 * @brief 硬件版本识别：通过 PB5/PB6 读取 PCB 版本号
 *******************************************************************************
 */

#ifndef HW_VER_H__
#define HW_VER_H__

#include <stdint.h>

// PB5
#define HW_VERSION_0_PORT           (GPIO_PORT_B)
#define HW_VERSION_0_PIN            (GPIO_PIN_05)

// PB6
#define HW_VERSION_1_PORT           (GPIO_PORT_B)
#define HW_VERSION_1_PIN            (GPIO_PIN_06)

/**
 * @brief  硬件版本引脚初始化（上拉输入）
 * @note   系统启动时调用一次即可
 */
void hw_version_init(void);

/**
 * @brief  获取硬件版本号
 * @retval 0~3，由 HW_VER[1:0] (PB6:PB5) 组成
 */
uint8_t hw_version_get(void);

#endif /* HW_VER_H__ */
