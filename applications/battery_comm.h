#ifndef BATTERY_COMM_H__
#define BATTERY_COMM_H__

#include <stdint.h>

/* 电池实例索引 */
#define BAT1    0U
#define BAT2    1U

#define BAT_INDEX_SN                0x00005080UL
#define BAT_INDEX_BMS_SW_VERSION    0x00005081UL
#define BAT_INDEX_BMS_HW_VERSION    0x00005082UL

/* CAN 帧分发入口：由 CanPort_Task 调用，内部按 can_id 选实例、按 CMD 字节分派解析 */
void BatteryComm_ParseCanFrame(uint8_t bat_idx, const uint8_t *rx_buf, uint8_t len);

/* 命令发送：bat_idx=BAT1/BAT2，0=成功，-1=失败 */
int BatteryComm_SendGetVersion(uint8_t bat_idx);
int BatteryComm_SendIndexQuery(uint8_t bat_idx, uint32_t query_index);
int BatteryComm_SendReset(uint8_t bat_idx);

#endif /* BATTERY_COMM_H__ */
