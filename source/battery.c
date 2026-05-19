#include "battery.h"

#include "FreeRTOS.h"
#include "task.h"

#include "can.h"
#include "uart.h"

#define BATTERY_CAN_ID_STD              0x41U

/* 各命令帧的 byte0..byte6 定义
 复位命令
 查询电池电压
 查询电池版本
 */
static const uint8_t BAT_CMD_RESET[7]         = { 0x82, 0x97, 0x00, 0x00, 0x00, 0x00, 0x01 };
static const uint8_t BAT_CMD_VOLT_QUERY[7]    = { 0x00, 0x82, 0x02, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t BAT_CMD_VERSION_QUERY[7] = { 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00 };


/* 协议校验：对校验字节之前的有效载荷逐字节异或。 */
static uint8_t Battery_CalcXorChecksum(const uint8_t *buf, uint8_t len)
{
    uint8_t cs = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

/* 通用 CAN 帧发送：payload 为 byte0..byte6，byte7 自动计算 XOR 校验 */
static int Battery_SendFrame(uint32_t can_id, const uint8_t payload[7])
{
    uint8_t tx_buf[8];
    uint8_t i;

    for (i = 0U; i < 7U; i++) {
        tx_buf[i] = payload[i];
    }
    tx_buf[7] = Battery_CalcXorChecksum(tx_buf, 7U);

    return can_send_std_frame(can_id, tx_buf, 8U);
}

/* -----------------------------------------------------------------------
 * 0x13  电池版本应答帧解析（上行被动）
 *   Byte[0]=0x13
 *   Byte[1]=主版本号  Byte[2]=子版本号  Byte[3]=修订本号
 *   Byte[4]=0x00      Byte[5]=0x00      Byte[6]=CAN_ID
 *   Byte[7]=XOR校验
 * ----------------------------------------------------------------------- */
void Battery_ParseVersionReply(const uint8_t *rx_buf, uint8_t len)
{
    uint8_t cs_calc;
    uint8_t major_ver;
    uint8_t minor_ver;
    uint8_t patch_ver;
    uint8_t bat_can_id;

    if (len < 8U) {
        Uart_Printf("[BAT] version reply too short (%u bytes)\r\n",
                    (unsigned)len);
        return;
    }

    if (rx_buf[0] != 0x13U) {
        Uart_Printf("[BAT] version reply CMD mismatch (0x%02X)\r\n",
                    (unsigned)rx_buf[0]);
        return;
    }

    /* 校验 byte7 = XOR(byte0..byte6) */
    cs_calc = Battery_CalcXorChecksum(rx_buf, 7U);
    if (cs_calc != rx_buf[7]) {
        Uart_Printf("[BAT] version reply checksum error (calc=0x%02X got=0x%02X)\r\n",
                    (unsigned)cs_calc, (unsigned)rx_buf[7]);
        return;
    }

    major_ver  = rx_buf[1];
    minor_ver  = rx_buf[2];
    patch_ver  = rx_buf[3];
    bat_can_id = rx_buf[6];

    Uart_Printf("[BAT] version reply: V%u.%u.%u  CAN_ID=0x%02X\r\n",
                (unsigned)major_ver, (unsigned)minor_ver,
                (unsigned)patch_ver, (unsigned)bat_can_id);
}

void Battery_Task(void *param)
{
    int ret;

    (void)param;

    // 复位指令
    ret = Battery_SendFrame(BATTERY_CAN_ID_STD, BAT_CMD_RESET);
    if (0 == ret) {
        Uart_Printf("[BAT] reset TX ok (ID 0x%02lX)\r\n",
                    (unsigned long)BATTERY_CAN_ID_STD);
    } else {
        Uart_Printf("[BAT] reset TX fail (ID 0x%02lX)\r\n",
                    (unsigned long)BATTERY_CAN_ID_STD);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;) {
        // 间隔2s发送版本查询指令 --- 心跳
        ret = Battery_SendFrame(BATTERY_CAN_ID_STD, BAT_CMD_VERSION_QUERY);
        if (0 == ret) {
            Uart_Printf("[BAT] version query TX ok\r\n");
        } else {
            Uart_Printf("[BAT] version query TX fail\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
