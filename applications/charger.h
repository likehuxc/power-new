#ifndef CHARGER_H__
#define CHARGER_H__

#include <stdint.h>

/* CAN 帧分发入口：由 CanPort_Task 调用 */
void Charger_ParseCanFrame(uint32_t can_id, const uint8_t *rx_buf, uint8_t len);

#endif /* CHARGER_H__ */
