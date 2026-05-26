#ifndef BATTERY_H__
#define BATTERY_H__

#include <stdint.h>

void Battery_Task(void *param);

/* CAN 帧分发入口：由 CanPort_Task 调用，内部按 CMD 字节分派解析 */
void Battery_ParseCanFrame(uint32_t can_id, const uint8_t *rx_buf, uint8_t len);

/* 命令发送：0=成功，-1=失败 */
int Battery_Send_GetVersion(void);
int Battery_Send_GetFullCapacity(void);
int Battery_Send_GetRemainCapacity(void);
int Battery_Send_GetVoltage(void);
int Battery_Send_GetCurrent(void);
int Battery_Send_GetSN(void);
int Battery_Send_Reset(void);

#endif /* BATTERY_H__ */
