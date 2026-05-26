/**
 *******************************************************************************
 * @file  bmi088.c
 * @brief BMI088 六轴 IMU 驱动（I2C 轮询模式）
 *
 * @note  关键设计要点：
 *        1. BMI088 内部是两颗独立芯片，加速度计和陀螺仪有各自的 I2C 地址
 *        2. 上电后加速度计默认处于 Suspend 模式，必须主动唤醒
 *        3. 在 Suspend 模式下两次写入间隔必须 >= 450µs
 *        4. 正常模式下两次写入间隔必须 >= 2µs
 *******************************************************************************
 */

#include "bmi088.h"
#include "i2c.h"
#include "hc32_ll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

/*******************************************************************************
 * I2C 地址定义（7 位地址，SDO1/SDO2 接 GND）
 ******************************************************************************/
#define BMI088_ACC_ADDR         (0x18U)     /* 加速度计 I2C 地址 (SDO1=GND) */
#define BMI088_GYRO_ADDR        (0x68U)     /* 陀螺仪 I2C 地址 (SDO2=GND) */

/*******************************************************************************
 * 加速度计寄存器地址
 ******************************************************************************/
#define ACC_CHIP_ID             (0x00U)     /* 加速度计芯片 ID */
#define ACC_ERR_REG             (0x02U)     /* 加速度计错误寄存器 */
#define ACC_STATUS              (0x03U)     /* 加速度计状态寄存器 */
#define ACC_X_LSB               (0x12U)      /* 加速度计 X 轴 */
#define ACC_X_MSB               (0x13U)     
#define ACC_Y_LSB               (0x14U)      /* 加速度计 Y 轴 */
#define ACC_Y_MSB               (0x15U)
#define ACC_Z_LSB               (0x16U)      /* 加速度计 Z 轴 */
#define ACC_Z_MSB               (0x17U)
#define ACC_SENSORTIME_0        (0x18U)      /* 加速度计计时时间 */
#define ACC_INT_STAT_1          (0x1DU)
#define TEMP_MSB                (0x22U)      /* 温度 MSB */
#define TEMP_LSB                (0x23U)      /* 温度 LSB */
#define ACC_CONF                (0x40U)      /* 加速度计配置 */
#define ACC_RANGE               (0x41U)      /* 加速度计量程 */
#define ACC_PWR_CONF            (0x7CU)      /* 加速度计电源配置 */
#define ACC_PWR_CTRL            (0x7DU)      /* 加速度计电源控制 */
#define ACC_SOFTRESET           (0x7EU)      /* 加速度计软复位 */

/*******************************************************************************
 * 陀螺仪寄存器地址
 ******************************************************************************/
#define GYRO_CHIP_ID            (0x00U)      /* 陀螺仪芯片 ID */
#define GYRO_X_LSB              (0x02U)      /* 陀螺仪 X 轴 */
#define GYRO_X_MSB              (0x03U) 
#define GYRO_Y_LSB              (0x04U)      /* 陀螺仪 Y 轴 */
#define GYRO_Y_MSB              (0x05U)
#define GYRO_Z_LSB              (0x06U)      /* 陀螺仪 Z 轴 */
#define GYRO_Z_MSB              (0x07U)
#define GYRO_INT_STAT_1         (0x0AU)      /* 陀螺仪中断状态 1 */
#define GYRO_RANGE              (0x0FU)      /* 陀螺仪量程 */
#define GYRO_BANDWIDTH          (0x10U)      /* 陀螺仪带宽 */
#define GYRO_LPM1               (0x11U)      /* 陀螺仪低功耗模式 1 */
#define GYRO_SOFTRESET          (0x14U)      /* 陀螺仪软复位 */
#define GYRO_INT_CTRL           (0x15U)      /* 陀螺仪中断控制 */
#define GYRO_INT3_INT4_IO_CONF  (0x16U)      /* 陀螺仪中断 3 和 4 IO 配置 */
#define GYRO_INT3_INT4_IO_MAP   (0x18U)      /* 陀螺仪中断 3 和 4 IO 映射 */

/*******************************************************************************
 * 芯片 ID 期望值
 ******************************************************************************/
#define BMI088_ACC_CHIP_ID_VAL  (0x1EU)
#define BMI088_GYRO_CHIP_ID_VAL (0x0FU)

/*******************************************************************************
 * 量程配置（当前使用的配置）
 ******************************************************************************/
/* 加速度计: ±6g, ODR=100Hz, Normal BW */
#define ACC_RANGE_CFG           (0x01U)     /* ±6g */
#define ACC_CONF_CFG            (0xA8U)     /* Normal BW, ODR=100Hz 不滤波 100个数据/S */

/* 陀螺仪: ±2000°/s, ODR=1000Hz, BW=116Hz */
#define GYRO_RANGE_CFG          (0x00U)     /* ±2000°/s */
#define GYRO_BW_CFG             (0x02U)     /* ODR=1000Hz, BW=116Hz */

/*******************************************************************************
 * 灵敏度常量
 ******************************************************************************/
/* 加速度计: ±6g → 5460 LSB/g → 1 LSB = 9.80665/5460 m/s² */
#define ACC_SENSITIVITY_6G      (5460.0f)
#define GRAVITY                 (9.80665f)

/* 陀螺仪: ±2000°/s → 16.384 LSB/(°/s) → 1 LSB = 1/16.384 °/s */
#define GYRO_SENSITIVITY_2000   (16.384f)

/*******************************************************************************
 * 内部辅助函数
 ******************************************************************************/

/**
 * @brief  向加速度计写单字节
 */
static int32_t acc_write_reg(uint8_t reg, uint8_t val)
{
    return i2c_write(&g_hi2c2, BMI088_ACC_ADDR, reg, 1U, &val, 1U);
}

/**
 * @brief  从加速度计读数据
 */
static int32_t acc_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return i2c_read(&g_hi2c2, BMI088_ACC_ADDR, reg, 1U, buf, len);
}

/**
 * @brief  向陀螺仪写单字节
 */
static int32_t gyro_write_reg(uint8_t reg, uint8_t val)
{
    return i2c_write(&g_hi2c2, BMI088_GYRO_ADDR, reg, 1U, &val, 1U);
}

/**
 * @brief  从陀螺仪读数据
 */
static int32_t gyro_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return i2c_read(&g_hi2c2, BMI088_GYRO_ADDR, reg, 1U, buf, len);
}

/*******************************************************************************
 * 加速度计初始化
 ******************************************************************************/
static int32_t bmi088_acc_init(void)
{
    int32_t ret;
    uint8_t chip_id = 0;

    /* 1. 软复位加速度计 */
    (void)acc_write_reg(ACC_SOFTRESET, 0xB6U);
    /* 复位后需等待 10ms 启动时间 */
    DDL_DelayMS(10U);

    /* 2. 验证芯片 ID */
    ret = acc_read_reg(ACC_CHIP_ID, &chip_id, 1U);
    if (ret != LL_OK) {
        return BMI088_ERR_COMM;
    }
    if (chip_id != BMI088_ACC_CHIP_ID_VAL) {
        LOG_ERROR("BMI088 ACC chip_id=0x%02X, expect 0x%02X", chip_id, BMI088_ACC_CHIP_ID_VAL);
        return BMI088_ERR_CHIP_ID;
    }

    /*
     * 3. 唤醒加速度计
     *    上电/复位后加速度计处于 Suspend 模式，此模式下两次写入间隔需 >= 450µs
     *    先写 ACC_PWR_CTRL = 0x04 使能加速度计
     *    再写 ACC_PWR_CONF = 0x00 进入 Active 模式
     */
    ret = acc_write_reg(ACC_PWR_CTRL, 0x04U);
    if (ret != LL_OK) return BMI088_ERR_COMM;
    /* Suspend 模式下写入间隔需 >= 450µs，这里等 1ms 确保安全 */
    vTaskDelay(pdMS_TO_TICKS(1U));

    ret = acc_write_reg(ACC_PWR_CONF, 0x00U);
    if (ret != LL_OK) return BMI088_ERR_COMM;
    vTaskDelay(pdMS_TO_TICKS(1U));

    /* 4. 配置量程: ±6g */
    ret = acc_write_reg(ACC_RANGE, ACC_RANGE_CFG);
    if (ret != LL_OK) return BMI088_ERR_COMM;
    DDL_DelayUS(10U);

    /* 5. 配置 ODR 和带宽: Normal BW, ODR=100Hz */
    ret = acc_write_reg(ACC_CONF, ACC_CONF_CFG);
    if (ret != LL_OK) return BMI088_ERR_COMM;
    DDL_DelayUS(10U);

    LOG_INFO("BMI088 ACC init OK, chip_id=0x%02X", chip_id);
    return BMI088_OK;
}

/*******************************************************************************
 * 陀螺仪初始化
 ******************************************************************************/
static int32_t bmi088_gyro_init(void)
{
    int32_t ret;
    uint8_t chip_id = 0;

    /* 1. 软复位陀螺仪 */
    (void)gyro_write_reg(GYRO_SOFTRESET, 0xB6U);
    /* 陀螺仪复位后需等待 10ms */
    DDL_DelayMS(10U);

    /* 2. 验证芯片 ID */
    ret = gyro_read_reg(GYRO_CHIP_ID, &chip_id, 1U);
    if (ret != LL_OK) {
        return BMI088_ERR_COMM;
    }
    if (chip_id != BMI088_GYRO_CHIP_ID_VAL) {
        LOG_ERROR("BMI088 GYRO chip_id=0x%02X, expect 0x%02X", chip_id, BMI088_GYRO_CHIP_ID_VAL);
        return BMI088_ERR_CHIP_ID;
    }

    /* 3. 配置量程: ±2000°/s */
   ret = gyro_write_reg(GYRO_RANGE, GYRO_RANGE_CFG);
   if (ret != LL_OK) return BMI088_ERR_COMM;
   DDL_DelayMS(10U);

    /* 4. 配置带宽: ODR=1000Hz, BW=116Hz */
    ret = gyro_write_reg(GYRO_BANDWIDTH, GYRO_BW_CFG);
    if (ret != LL_OK) return BMI088_ERR_COMM;
    DDL_DelayMS(10U);

    /* 5. 确保处于正常模式（模式切换后需等待 30ms） */
    ret = gyro_write_reg(GYRO_LPM1, 0x00U);
    if (ret != LL_OK) return BMI088_ERR_COMM;
    DDL_DelayMS(30U);

    LOG_INFO("BMI088 GYRO init OK, chip_id=0x%02X", chip_id);
    return BMI088_OK;
}

/*******************************************************************************
 * 公共接口实现
 ******************************************************************************/

int32_t BMI088_Init(void)
{
    int32_t ret;

//    /* 初始化加速度计 */
//    ret = bmi088_acc_init();
//    if (ret != BMI088_OK) {
//        return ret;
//    }

    /* 初始化陀螺仪 */
    ret = bmi088_gyro_init();
    if (ret != BMI088_OK) {
        return ret;
    }

    return BMI088_OK;
}

int32_t BMI088_ReadAccelRaw(bmi088_accel_raw_t *raw)
{
    int32_t ret;
    uint8_t buf[6];

    if (raw == NULL) {
        return BMI088_ERR_PARAM;
    }

    /* 从 0x12 连续读 6 字节 (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB) */
    ret = acc_read_reg(ACC_X_LSB, buf, 6U);
    if (ret != LL_OK) {
        return BMI088_ERR_COMM;
    }

    raw->x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    raw->y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    raw->z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    return BMI088_OK;
}

int32_t BMI088_ReadGyroRaw(bmi088_gyro_raw_t *raw)
{
    int32_t ret;
    uint8_t buf[6];

    if (raw == NULL) {
        return BMI088_ERR_PARAM;
    }

    /* 从 0x02 连续读 6 字节 (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB) */
    ret = gyro_read_reg(GYRO_X_LSB, buf, 6U);
    if (ret != LL_OK) {
        return BMI088_ERR_COMM;
    }

    raw->x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    raw->y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    raw->z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    return BMI088_OK;
}

int32_t BMI088_ReadTemperature(float *temp)
{
    int32_t ret;
    uint8_t buf[2];
    int16_t temp_raw;

    if (temp == NULL) {
        return BMI088_ERR_PARAM;
    }

    /* 温度寄存器在加速度计地址空间: 0x22(MSB), 0x23(LSB) */
    ret = acc_read_reg(TEMP_MSB, buf, 2U);
    if (ret != LL_OK) {
        return BMI088_ERR_COMM;
    }

    /*
     * 温度计算:
     * Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32)
     * 如果 > 1023 则减 2048
     * Temperature = Temp_int11 * 0.125 + 23
     */
    uint16_t temp_uint11 = ((uint16_t)buf[0] << 3) | (buf[1] >> 5);
    if (temp_uint11 > 1023U) {
        temp_raw = (int16_t)temp_uint11 - 2048;
    } else {
        temp_raw = (int16_t)temp_uint11;
    }
    *temp = (float)temp_raw * 0.125f + 23.0f;

    return BMI088_OK;
}

int32_t BMI088_ReadAll(bmi088_data_t *data)
{
    int32_t ret;

    if (data == NULL) {
        return BMI088_ERR_PARAM;
    }

    /* 读取加速度原始值 */
    ret = BMI088_ReadAccelRaw(&data->accel_raw);
    if (ret != BMI088_OK) return ret;

    /* 读取陀螺仪原始值 */
    ret = BMI088_ReadGyroRaw(&data->gyro_raw);
    if (ret != BMI088_OK) return ret;

    /* 读取温度 */
    ret = BMI088_ReadTemperature(&data->temperature);
    if (ret != BMI088_OK) return ret;

    /* 转换加速度为 m/s²: raw / sensitivity * g */
    data->accel.x = (float)data->accel_raw.x / ACC_SENSITIVITY_6G * GRAVITY;
    data->accel.y = (float)data->accel_raw.y / ACC_SENSITIVITY_6G * GRAVITY;
    data->accel.z = (float)data->accel_raw.z / ACC_SENSITIVITY_6G * GRAVITY;

    /* 转换角速度为 °/s: raw / sensitivity */
    data->gyro.x = (float)data->gyro_raw.x / GYRO_SENSITIVITY_2000;
    data->gyro.y = (float)data->gyro_raw.y / GYRO_SENSITIVITY_2000;
    data->gyro.z = (float)data->gyro_raw.z / GYRO_SENSITIVITY_2000;

    return BMI088_OK;
}

/*******************************************************************************
 * 测试任务
 ******************************************************************************/
void BMI088_Task(void *pvParameters)
{
    (void)pvParameters;

    /* 初始化 I2C2 */
    i2c2_init();

    bmi088_data_t imu;

    /* 初始化 BMI088 */
    if (BMI088_Init() != BMI088_OK) {
        LOG_ERROR("BMI088 init FAILED");
    }

    for (;;) {
        if (BMI088_ReadAll(&imu) == BMI088_OK) {
            LOG_INFO("ACC: %.2f, %.2f, %.2f m/s2 | GYRO: %.1f, %.1f, %.1f dps | T:%.1fC",
                     imu.accel.x, imu.accel.y, imu.accel.z,
                     imu.gyro.x, imu.gyro.y, imu.gyro.z,
                     imu.temperature);
        } else {
            LOG_ERROR("BMI088 read FAILED");
        }

        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}
