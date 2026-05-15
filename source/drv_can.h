/**
 *******************************************************************************
 * @file  drv_can.h
 *******************************************************************************
 */

#ifndef DRV_CAN_H__
#define DRV_CAN_H__

#include <stdint.h>

/* CAN 数据大小最大值 */
#define DRV_CAN_DATA_SIZE_MAX 8U

/* CAN 帧结构体 
 * u32ID: CAN 帧 ID，11位标准ID或29位扩展ID
 * u8IDE: CAN 帧 IDE，0表示标准ID，1表示扩展ID
 * u8DLC: CAN 帧 DLC，0-8表示数据长度，0表示远程帧，1-8表示数据帧
 * u8SelfTx: CAN 帧自发送标志，0表示不是自发送，1表示是自发送
 * au8Data: CAN 帧数据，最大8字节
 */
typedef struct {
    uint32_t u32ID;
    uint8_t  u8IDE;
    uint8_t  u8DLC;
    uint8_t  u8SelfTx;
    uint8_t  au8Data[DRV_CAN_DATA_SIZE_MAX];
} stc_drv_can_frame_t;

/* 初始化 CAN 外设 */
int32_t DrvCan_Init(void);
int32_t DrvCan_Send(const stc_drv_can_frame_t *pstcFrame);
int32_t DrvCan_Read(stc_drv_can_frame_t *pstcFrame);

#endif /* DRV_CAN_H__ */
