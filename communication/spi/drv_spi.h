/**
 *******************************************************************************
 * @file  drv_spi.h
 * @brief 硬件 SPI 主机驱动接口（DMA 模式，软件 CS）
 *        PD8=MOSI(func40), PD9=MISO(func41), PD10=CS(软件), PD11=SCK(func43)
 *        DMA1 CH2=TX, CH3=RX
 *******************************************************************************
 */

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "hc32_ll.h"

/*******************************************************************************
 * 公共 API
 ******************************************************************************/

/**
 * @brief  初始化 SPI1 及 DMA 通道
 */
void spi_init(void);

/**
 * @brief  向从机写数据（先拉低 CS，写完拉高 CS）
 * @param  reg    寄存器地址（1 字节，先发）
 * @param  buf    数据缓冲区
 * @param  len    字节数
 * @retval LL_OK
 */
int32_t spi_write(uint8_t reg, const uint8_t *buf, uint32_t len);

/**
 * @brief  从从机读数据（先拉低 CS，读完拉高 CS）
 * @param  reg    寄存器地址（1 字节，先发）
 * @param  buf    接收缓冲区
 * @param  len    字节数
 * @retval LL_OK
 */
int32_t spi_read(uint8_t reg, uint8_t *buf, uint32_t len);

#endif /* __DRV_SPI_H__ */
