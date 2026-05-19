#ifndef BATTERY_H__
#define BATTERY_H__

#include <stdint.h>

void Battery_Task(void *param);

/* 供 CAN 接收回调调用：解析 0x13 版本应答帧。 */
void Battery_ParseVersionReply(const uint8_t *rx_buf, uint8_t len);

#endif /* BATTERY_H__ */
