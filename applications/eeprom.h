/**
 *******************************************************************************
 * @file  eeprom.h
 * @brief EEPROM (GT24C32B) 读写驱动接口（基于硬件 I2C）
 *******************************************************************************
 */

#ifndef EEPROM_H__
#define EEPROM_H__

#include <stdint.h>
#include <string.h>

/* 返回值定义 */
#define EEPROM_OK           (0)
#define EEPROM_ERR_PARAM    (-1)
#define EEPROM_ERR_TIMEOUT  (-2)
#define EEPROM_ERR_BUSY     (-3)

/*******************************************************************************
 * EEPROM 基础读写接口
 ******************************************************************************/

/**
 * @brief  初始化 EEPROM 模块（内部调用硬件 I2C 初始化）
 */
void Eeprom_Init(void);
void Eeprom_Task(void *pvParameters);

/**
 * @brief  连续写（自动分页，避免跨页回卷）
 * @param  mem_addr  EEPROM 内部地址（0 ~ 4095）
 * @param  buf       待写入数据缓冲区
 * @param  len       字节数
 * @retval EEPROM_OK / EEPROM_ERR_PARAM / EEPROM_ERR_TIMEOUT
 */
int32_t Eeprom_Write(uint16_t mem_addr, const uint8_t *buf, uint16_t len);

/**
 * @brief  连续读
 * @param  mem_addr  EEPROM 内部地址（0 ~ 4095）
 * @param  buf       接收缓冲区
 * @param  len       字节数
 * @retval EEPROM_OK / EEPROM_ERR_PARAM / EEPROM_ERR_TIMEOUT
 */
int32_t Eeprom_Read(uint16_t mem_addr, uint8_t *buf, uint16_t len);

/**
 * @brief  写单字节
 */
int32_t Eeprom_WriteByte(uint16_t mem_addr, uint8_t data);

/**
 * @brief  读单字节
 */
int32_t Eeprom_ReadByte(uint16_t mem_addr, uint8_t *data);

#endif /* EEPROM_H__ */
