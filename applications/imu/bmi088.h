/**
 *******************************************************************************
 * @file  bmi088.h
 * @brief BMI088 六轴 IMU 驱动接口（加速度计 + 陀螺仪，I2C 模式）
 *
 * @note  BMI088 内部是两颗独立芯片，拥有独立的 I2C 地址：
 *        - 加速度计: 0x18 (SDO1=GND) / 0x19 (SDO1=VDDIO)
 *        - 陀螺仪:   0x68 (SDO2=GND) / 0x69 (SDO2=VDDIO)
 *******************************************************************************
 */

#ifndef BMI088_H__
#define BMI088_H__

#include <stdint.h>

/*******************************************************************************
 * 返回值定义
 ******************************************************************************/
#define BMI088_OK               (0)
#define BMI088_ERR_PARAM        (-1)
#define BMI088_ERR_COMM         (-2)
#define BMI088_ERR_CHIP_ID      (-3)
#define BMI088_ERR_TIMEOUT      (-4)

/*******************************************************************************
 * 数据结构
 ******************************************************************************/

/** 三轴加速度原始数据 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi088_accel_raw_t;

/** 三轴陀螺仪原始数据 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi088_gyro_raw_t;

/** 三轴浮点数据（物理量） */
typedef struct {
    float x;
    float y;
    float z;
} bmi088_data_f_t;

/** IMU 完整数据 */
typedef struct {
    bmi088_accel_raw_t accel_raw;    /* 加速度原始值 */
    bmi088_gyro_raw_t  gyro_raw;     /* 陀螺仪原始值 */
    bmi088_data_f_t    accel;        /* 加速度，单位 m/s² */
    bmi088_data_f_t    gyro;         /* 角速度，单位 °/s */
    float              temperature;  /* 温度，单位 °C */
} bmi088_data_t;

/*******************************************************************************
 * 公共接口
 ******************************************************************************/

/**
 * @brief  初始化 BMI088（加速度计 + 陀螺仪）
 * @retval BMI088_OK / BMI088_ERR_CHIP_ID / BMI088_ERR_COMM
 */
int32_t BMI088_Init(void);

/**
 * @brief  读取加速度计原始数据
 * @param  raw  输出结构体指针
 * @retval BMI088_OK / BMI088_ERR_COMM
 */
int32_t BMI088_ReadAccelRaw(bmi088_accel_raw_t *raw);

/**
 * @brief  读取陀螺仪原始数据
 * @param  raw  输出结构体指针
 * @retval BMI088_OK / BMI088_ERR_COMM
 */
int32_t BMI088_ReadGyroRaw(bmi088_gyro_raw_t *raw);

/**
 * @brief  读取温度传感器
 * @param  temp  输出温度值（°C）
 * @retval BMI088_OK / BMI088_ERR_COMM
 */
int32_t BMI088_ReadTemperature(float *temp);

/**
 * @brief  一次性读取所有数据（加速度 + 陀螺仪 + 温度），并转换为物理量
 * @param  data  输出数据结构体指针
 * @retval BMI088_OK / BMI088_ERR_COMM
 */
int32_t BMI088_ReadAll(bmi088_data_t *data);

/**
 * @brief  IMU 测试任务（FreeRTOS）
 */
void BMI088_Task(void *pvParameters);

#endif /* BMI088_H__ */
