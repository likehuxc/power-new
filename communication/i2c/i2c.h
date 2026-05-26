/**
 *******************************************************************************
 * @file  i2c.h
 * @brief 硬件 I2C 主机驱动接口（轮询模式，含总线恢复）
 *        支持多实例：通过 i2c_handle_t 句柄区分不同 I2C 外设
 *******************************************************************************
 */

#ifndef __I2C_H__
#define __I2C_H__

#include "hc32_ll.h"

/**
 * @brief I2C 实例硬件配置
 */
typedef struct {
    CM_I2C_TypeDef *unit;       /* I2C 外设单元，如 CM_I2C1 */
    uint32_t        fcg;        /* 外设时钟门控，如 FCG1_PERIPH_I2C1 */
    uint8_t         scl_port;   /* SCL GPIO 端口 */
    uint16_t        scl_pin;    /* SCL GPIO 引脚 */
    uint8_t         sda_port;   /* SDA GPIO 端口 */
    uint16_t        sda_pin;    /* SDA GPIO 引脚 */
    uint8_t         scl_func;   /* SCL 复用功能号 */
    uint8_t         sda_func;   /* SDA 复用功能号 */
    uint32_t        baudrate;   /* 通信速率，如 100000UL */
    uint32_t        clk_div;    /* 时钟分频，如 I2C_CLK_DIV2 */
    uint32_t        scl_time;   /* SCL 高/低电平时间补偿 */
    uint32_t        timeout;    /* 轮询超时计数 */
} i2c_config_t;

/**
 * @brief I2C 实例句柄
 */
typedef struct {
    const i2c_config_t *cfg;    /* 指向硬件配置（const，放 flash） */
} i2c_handle_t;

/*******************************************************************************
 * 公共 API
 ******************************************************************************/

int32_t i2c_init(i2c_handle_t *hi2c, const i2c_config_t *cfg);

int32_t i2c_write(i2c_handle_t *hi2c, uint16_t dev_addr, uint16_t reg_addr,
                  uint8_t reg_addr_len, const uint8_t *buf, uint32_t len);

int32_t i2c_read(i2c_handle_t *hi2c, uint16_t dev_addr, uint16_t reg_addr,
                 uint8_t reg_addr_len, uint8_t *buf, uint32_t len);

int32_t i2c_check_ack(i2c_handle_t *hi2c, uint16_t dev_addr);

int32_t i2c_recover(i2c_handle_t *hi2c);

/*******************************************************************************
 * 预定义实例（方便现有代码迁移）
 * 使用前需调用对应的 i2cX_init() 完成初始化
 ******************************************************************************/

extern i2c_handle_t g_hi2c1;
extern i2c_handle_t g_hi2c2;

/** @brief 使用默认配置初始化 I2C1 实例（PA11-SCL, PA10-SDA） */
int32_t i2c1_init(void);

/** @brief 使用默认配置初始化 I2C2 实例（PA9-SCL, PA8-SDA） */
int32_t i2c2_init(void);

#endif /* __I2C_H__ */
