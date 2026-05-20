#ifndef BATTERY_H__
#define BATTERY_H__

#include <stdint.h>

void Battery_Task(void *param);

/* CAN 帧分发入口：由 CanPort_Task 调用，内部按 CMD 字节分派解析 */
void Battery_ParseCanFrame(uint32_t can_id, const uint8_t *rx_buf, uint8_t len);

/* 公共查询 API */
int Battery_QueryVersion(void);
int Battery_QueryFullCapacity(void);
int Battery_QueryRemainCapacity(void);
int Battery_QueryVoltage(void);

#endif /* BATTERY_H__ */
