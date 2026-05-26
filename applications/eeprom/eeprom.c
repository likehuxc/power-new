/**
 *******************************************************************************
 * @file  eeprom.c
 * @brief EEPROM (GT24C32B) 读写驱动（基于硬件 I2C1 轮询模式）
 *
 * @note  GT24C32B: 32Kbit = 4KB，页大小 32 字节，7 位地址 0x50
 *******************************************************************************
 */

#include "eeprom.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

/*******************************************************************************
 * EEPROM 参数定义
 ******************************************************************************/
#define EEPROM_DEV_ADDR_7BIT    (0x50U)     /* A2/A1/A0 = 0 */
#define EEPROM_SIZE_BYTES       (4096U)     /* 4KB */
#define EEPROM_PAGE_SIZE        (32U)       /* 页大小 32 字节 */
#define EEPROM_WRITE_CYCLE_MS   (5U)        /* 内部写周期最大 5ms */
#define EEPROM_ACK_POLL_RETRY   (50U)       /* ACK 轮询最大重试次数 */

/*******************************************************************************
 * 内部辅助函数
 ******************************************************************************/

/**
 * @brief  等待 EEPROM 内部写周期完成（ACK Polling）
 * @note   写操作后 EEPROM 进入内部编程周期，此时不响应 ACK。
 *         通过反复发送设备地址检测 ACK 来判断写周期是否结束。
 * @retval EEPROM_OK / EEPROM_ERR_TIMEOUT
 */
static int32_t eeprom_wait_ready(void)
{
    for (uint32_t i = 0U; i < EEPROM_ACK_POLL_RETRY; i++) {
        if (i2c_check_ack(&g_hi2c1, EEPROM_DEV_ADDR_7BIT) == LL_OK) {
            return EEPROM_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return EEPROM_ERR_TIMEOUT;
}

/*******************************************************************************
 * 公共接口
 ******************************************************************************/

/**
 * @brief  EEPROM 测试任务
 * @param  pvParameters 任务参数指针
 */
void Eeprom_Task(void *pvParameters)
{
    (void)pvParameters; 

    /* 初始化 I2C1 */
    i2c1_init();

    /* 测试任务循环 */
    for (;;) {
        static uint8_t cnt = 0;
        uint8_t wbuf[4] = {cnt, cnt + 1, cnt + 2, cnt + 3};
        uint8_t rbuf[4] = {0};

        // 写入数据
        if (Eeprom_Write(0x0000, wbuf, 4) == EEPROM_OK) {
            LOG_INFO("Eeprom_Write: %02X %02X %02X %02X", wbuf[0], wbuf[1], wbuf[2], wbuf[3]);
        } else {
            LOG_ERROR("Eeprom_Write FAILED");
        }

        // 读取数据
        if (Eeprom_Read(0x0000, rbuf, 4) == EEPROM_OK) {
            LOG_INFO("Eeprom_Read : %02X %02X %02X %02X", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
        } else {
            LOG_ERROR("Eeprom_Read FAILED");
        }

        cnt++;
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

/**
 * @brief  连续写（自动分页，避免跨页回卷）
 * @param  mem_addr  EEPROM 内部地址（0 ~ 4095）
 * @param  buf       待写入数据缓冲区
 * @param  len       字节数
 * @retval EEPROM_OK / EEPROM_ERR_PARAM / EEPROM_ERR_TIMEOUT
 */
int32_t Eeprom_Write(uint16_t mem_addr, const uint8_t *buf, uint16_t len)
{
    int32_t ret;

    /* 参数校验 */
    if ((buf == NULL) || (len == 0U)) {
        return EEPROM_ERR_PARAM;
    }
    if (((uint32_t)mem_addr + (uint32_t)len) > EEPROM_SIZE_BYTES) {
        return EEPROM_ERR_PARAM;
    }

    while (len > 0U) {
        /*1. 计算当前页剩余空间 */
        uint16_t page_off  = (uint16_t)(mem_addr % EEPROM_PAGE_SIZE);
        uint16_t page_left = (uint16_t)(EEPROM_PAGE_SIZE - page_off);
        uint16_t chunk     = (len < page_left) ? len : page_left;

        /*2. 通过硬件 I2C 发送（2字节内存地址 + 数据） */
        ret = i2c_write(&g_hi2c1, EEPROM_DEV_ADDR_7BIT, mem_addr, 2U, buf, (uint32_t)chunk);
        if (ret != LL_OK) {
            return EEPROM_ERR_TIMEOUT;
        }

        /*3. 等待内部写周期完成 */
        ret = eeprom_wait_ready();
        if (ret != EEPROM_OK) {
            return ret;
        }

        /*4. 更新指针 */
        mem_addr = (uint16_t)(mem_addr + chunk);
        buf     += chunk;
        len      = (uint16_t)(len - chunk);
    }

    return EEPROM_OK;
}

/**
 * @brief  连续读
 * @param  mem_addr  EEPROM 内部地址（0 ~ 4095）
 * @param  buf       接收缓冲区
 * @param  len       字节数
 * @retval EEPROM_OK / EEPROM_ERR_PARAM / EEPROM_ERR_TIMEOUT
 *
 * @note   读时序：先写 2 字节内部地址，Restart 后读数据（标准随机读，单次事务）。
 */
int32_t Eeprom_Read(uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    int32_t ret;

    /* 参数校验 */
    if ((buf == NULL) || (len == 0U)) {
        return EEPROM_ERR_PARAM;
    }
    if (((uint32_t)mem_addr + (uint32_t)len) > EEPROM_SIZE_BYTES) {
        return EEPROM_ERR_PARAM;
    }

    ret = i2c_read(&g_hi2c1, EEPROM_DEV_ADDR_7BIT, mem_addr, 2U, buf, (uint32_t)len);
    if (ret != LL_OK) {
        return EEPROM_ERR_TIMEOUT;
    }

    return EEPROM_OK;
}

/**
 * @brief  写单字节
 * @param  mem_addr  EEPROM 内部地址
 * @param  data      待写入字节
 * @retval EEPROM_OK / EEPROM_ERR_TIMEOUT
 */
int32_t Eeprom_WriteByte(uint16_t mem_addr, uint8_t data)
{
    return Eeprom_Write(mem_addr, &data, 1U);
}

/**
 * @brief  读单字节
 * @param  mem_addr  EEPROM 内部地址
 * @param  data      接收指针
 * @retval EEPROM_OK / EEPROM_ERR_PARAM / EEPROM_ERR_TIMEOUT
 */
int32_t Eeprom_ReadByte(uint16_t mem_addr, uint8_t *data)
{
    if (data == NULL) {
        return EEPROM_ERR_PARAM;
    }
    return Eeprom_Read(mem_addr, data, 1U);
}
