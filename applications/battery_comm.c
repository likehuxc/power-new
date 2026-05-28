/**
 *******************************************************************************
 * @file  battery_comm.c
 * @brief 电池通信协议：CAN 命令发送、应答帧解析
 *
 * 协议概述：
 *   - 所有帧为标准 CAN 帧，8 字节，byte7 = byte0..byte6 的异或校验
 *   - 下行（主机→电池）：byte0=0x00 为请求前缀，byte1=命令码，byte2=子索引
 *   - 上行（电池→主机）：byte0=命令码，byte1=子索引，byte2..5=大端数据
 *
 * 节点：
 *   - 电池1 CAN ID 0x40（BAT1=0），电池2 CAN ID 0x41（BAT2=1），协议相同
 *
 * 支持的命令：
 *   CMD 0x82 0x97 : 复位
 *   CMD 0x13      : 版本查询/应答
 *   CMD 0x82      : 电池主动上报（满充容量、剩余容量、电压、电流）
 *   CMD 0x00/0x55 : 不定长索引查询/应答（SN/版本/生产日期/运行日志）
 *******************************************************************************
 */

#include "battery_comm.h"

#include <stddef.h>

#include "can_port.h"
#include "log.h"

#define BAT_COUNT           2U      // 电池数量
#define BAT_FRAME_LEN       8U      // 帧长度
#define BAT_PAYLOAD_LEN     7U      // 负载长度

#define BAT_CMD_VERSION     0x13U   // 查询版本命令
#define BAT_CMD_REPORT_DATA 0x82U   // 电池主动上报定长数据命令
#define BAT_CMD_QUERY_DATA  0x55U   // 不定长索引查询/应答命令

/* -----------------------------------------------------------------------
 * 命令与 payload 定义
 * 格式：byte0..byte6（共 7 字节），byte7 由 Battery_SendFrame() 自动计算 XOR 校验
 * ----------------------------------------------------------------------- */
static const uint8_t BAT_CMD_RESET[BAT_PAYLOAD_LEN]         = { 0x82, 0x97, 0x00, 0x00, 0x00, 0x00, 0x01 }; // 复位命令
static const uint8_t BAT_CMD_VERSION_QUERY[BAT_PAYLOAD_LEN] = { 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 查询版本命令
static const uint32_t BAT_CAN_ID[BAT_COUNT]                 = { 0x40U, 0x41U }; // 电池1和电池2的CAN ID

#define BAT_VAR_VALUE_MAX           80U // 不定长索引查询/应答命令的最大值
#define BAT_TEXT_VALUE_PRINT_MAX    32U

typedef struct {
    uint32_t reply_index;                // 本组 0x55 应答对应的 BAT_INDEX_*
    uint8_t data[BAT_VAR_VALUE_MAX];     // 数据缓冲区
    uint8_t expected_len;                // 期望长度
    uint8_t received_len;                // 已接收长度
    uint8_t crc[2];                      // CRC校验值
    uint8_t crc_len;                     // CRC长度
    uint8_t recv_start;                  // 是否已收到首帧并开始接收数据
    uint8_t next_seq;                    // 下一个序列号，用于校验序列号是否连续
} bat_var_rx_t;

static bat_var_rx_t s_var_rx[BAT_COUNT];

/* -----------------------------------------------------------------------
 * 内部工具函数
 * ----------------------------------------------------------------------- */

/* 计算 XOR 校验 */
static uint8_t Battery_CalcXorChecksum(const uint8_t *buf, uint8_t len)
{
    uint8_t cs = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

/* 读取32位大端数据 */
static uint32_t Battery_ReadU32BE(const uint8_t *buf)
{
    return (((uint32_t)buf[0]) << 24) |
           (((uint32_t)buf[1]) << 16) |
           (((uint32_t)buf[2]) <<  8) |
           (((uint32_t)buf[3]));
}

/* 解析版本数字 */
static uint8_t Battery_DecVersionDigit(uint8_t raw)
{
    if (raw >= 0x30U) {
        return (uint8_t)(raw - 0x30U);
    }
    return raw;
}

/* 计算CRC16校验 */
static uint16_t Battery_ModbusCrc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  b;

    for (i = 0U; i < len; i++) {
        crc ^= buf[i];
        for (b = 0U; b < 8U; b++) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* 检查XOR校验帧 */
static uint8_t Battery_CheckXorFrame(uint8_t idx, const uint8_t *buf, uint8_t len, const char *name)
{
    uint8_t cs_calc;

    if (len < BAT_FRAME_LEN) {
        LOG_WARN("[BAT%u] %s too short (%u)", (unsigned)(idx + 1U), name, (unsigned)len);
        return 0U;
    }

    cs_calc = Battery_CalcXorChecksum(buf, BAT_PAYLOAD_LEN);
    if (cs_calc != buf[7]) {
        LOG_WARN("[BAT%u] %s checksum err (calc=0x%02X got=0x%02X)",
                 (unsigned)(idx + 1U), name, (unsigned)cs_calc, (unsigned)buf[7]);
        return 0U;
    }

    return 1U;
}

/* 重置不定长索引查询/应答命令 */
static void Battery_VarRxReset(uint8_t idx)
{
    s_var_rx[idx].reply_index  = 0U;
    s_var_rx[idx].recv_start   = 0U;
    s_var_rx[idx].expected_len = 0U;
    s_var_rx[idx].received_len = 0U;
    s_var_rx[idx].crc_len      = 0U;
    s_var_rx[idx].next_seq     = 0U;
}

/* -----------------------------------------------------------------------
 * 发送层
 * ----------------------------------------------------------------------- */

static int Battery_SendFrame(uint8_t bat_idx, const uint8_t payload[BAT_PAYLOAD_LEN])
{
    uint32_t can_id;
    uint8_t  tx_buf[BAT_FRAME_LEN];
    uint8_t  i;

    can_id = BAT_CAN_ID[bat_idx];
    for (i = 0U; i < BAT_PAYLOAD_LEN; i++) {
        tx_buf[i] = payload[i];
    }
    tx_buf[7] = Battery_CalcXorChecksum(tx_buf, BAT_PAYLOAD_LEN);
    LOG_DEBUG("[BAT%u] tran ID:0x%02lX DATA: %02X %02X %02X %02X %02X %02X %02X %02X",
              (unsigned)(bat_idx + 1U), (unsigned long)can_id,
              tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
              tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]);
    return CanPort_Send(can_id, tx_buf, BAT_FRAME_LEN);
}

/* 发送版本查询命令 */
int BatteryComm_SendGetVersion(uint8_t bat_idx)
{
    LOG_DEBUG("[BAT%u] send GetVersion", (unsigned)(bat_idx + 1U));
    return Battery_SendFrame(bat_idx, BAT_CMD_VERSION_QUERY);
}

/* 发送复位命令 */
int BatteryComm_SendReset(uint8_t bat_idx)
{
    LOG_DEBUG("[BAT%u] send Reset", (unsigned)(bat_idx + 1U));
    return Battery_SendFrame(bat_idx, BAT_CMD_RESET);
}

/* 发送不定长索引查询命令 
举例：0x00, 0x55, 0x00, 0x00, 0x50, 0x80, 0x41 表示 查询索引为 0x5080 的电池1的数据
*/
int BatteryComm_SendIndexQuery(uint8_t bat_idx, uint32_t query_index)
{
    uint8_t payload[BAT_PAYLOAD_LEN];

    Battery_VarRxReset(bat_idx);
    s_var_rx[bat_idx].reply_index = query_index;

    payload[0] = 0x00U;
    payload[1] = BAT_CMD_QUERY_DATA;
    payload[2] = (uint8_t)(query_index >> 24);
    payload[3] = (uint8_t)(query_index >> 16);
    payload[4] = (uint8_t)(query_index >> 8);
    payload[5] = (uint8_t)query_index;
    payload[6] = (uint8_t)BAT_CAN_ID[bat_idx];

    LOG_DEBUG("[BAT%u] send query_index 0x%04lX",
              (unsigned)(bat_idx + 1U), (unsigned long)(query_index & 0xFFFFUL));
    return Battery_SendFrame(bat_idx, payload);
}

/* -----------------------------------------------------------------------
 * 应答帧解析（内部，均以 idx 区分实例）
 * ----------------------------------------------------------------------- */

static void Battery_ParseVersionReply(uint8_t idx, const uint8_t *rx_buf, uint8_t len)
{
    uint8_t v_b1_ma, v_b1_mi, v_b1_re, v_b2_mi;

    if (0U == Battery_CheckXorFrame(idx, rx_buf, len, "version reply")) {
        return;
    }
    // 解析版本数字
    v_b1_ma = Battery_DecVersionDigit(rx_buf[1]);
    v_b1_mi = Battery_DecVersionDigit(rx_buf[2]);
    v_b1_re = Battery_DecVersionDigit(rx_buf[3]);
    v_b2_mi = Battery_DecVersionDigit(rx_buf[4]);

    LOG_INFO("[BAT%u][VER] (单版本): V%u.%u.%u",
             (unsigned)(idx + 1U), (unsigned)v_b1_ma, (unsigned)v_b1_mi, (unsigned)v_b1_re);
}

static void Battery_ParseDataReport(uint8_t idx, const uint8_t *rx_buf, uint8_t len)
{
    uint8_t  sub_index;
    uint32_t raw_u32;
    int32_t  raw_i32;

    if (0U == Battery_CheckXorFrame(idx, rx_buf, len, "data report")) {
        return;
    }

    sub_index = rx_buf[1];
    raw_u32   = Battery_ReadU32BE(&rx_buf[2]);

    switch (sub_index) {
    case 0x00U:
        LOG_INFO("[BAT%u] full capacity: %lu mAh", (unsigned)(idx + 1U), (unsigned long)raw_u32);
        break;
    case 0x01U:
        LOG_INFO("[BAT%u] remain capacity: %lu mAh", (unsigned)(idx + 1U), (unsigned long)raw_u32);
        break;
    case 0x02U:
        raw_i32 = (int32_t)raw_u32;
        LOG_INFO("[BAT%u] voltage: %ld.%03ld V",
                 (unsigned)(idx + 1U),
                 (long)(raw_i32 / 1000),
                 (long)(raw_i32 >= 0 ? raw_i32 % 1000 : -(raw_i32 % 1000)));
        break;
    case 0x0EU:
        raw_i32 = (int32_t)raw_u32;
        LOG_INFO("[BAT%u] current: %ld mA", (unsigned)(idx + 1U), (long)raw_i32);
        break;
    default:
        break;
    }
}

/* 解析不定长索引查询/应答命令 */
static void Battery_OnVarValueDone(uint8_t idx, uint32_t reply_index, const uint8_t *data, uint8_t data_len)
{
    const char *name;
    uint8_t print_len;

    switch (reply_index) {
    case BAT_INDEX_SN:
        name = "SN";
        break;
    case BAT_INDEX_BMS_SW_VERSION:
        name = "BMS SW version";
        break;
    case BAT_INDEX_BMS_HW_VERSION:
        name = "BMS HW version";
        break;
    default:
        return;
    }

    print_len = (data_len > BAT_TEXT_VALUE_PRINT_MAX) ? BAT_TEXT_VALUE_PRINT_MAX : data_len;
    LOG_INFO("[BAT%u] %s: %.*s", (unsigned)(idx + 1U), name, (int)print_len, (const char *)data);
}

/* 解析不定长索引查询/应答命令 */
static void Battery_ParseVarReply(uint8_t idx, const uint8_t *frame, uint8_t len)
{
    bat_var_rx_t *rx = &s_var_rx[idx];
    uint8_t        i;

    if ((len < BAT_FRAME_LEN) || (BAT_CMD_QUERY_DATA != frame[0])) {
        return;
    }

    //1. 第一包数据：返回查询的索引
    if (0U == frame[1]) {
        uint32_t hdr_reply_index = Battery_ReadU32BE(&frame[2]);

        if (((0U != rx->reply_index) && (hdr_reply_index != rx->reply_index)) ||
            (0U == frame[7]) || (frame[7] > BAT_VAR_VALUE_MAX)) {
            LOG_WARN("[BAT%u] 0x55 header invalid reply_index=0x%04lX len=%u",
                     (unsigned)(idx + 1U), (unsigned long)(hdr_reply_index & 0xFFFFUL), (unsigned)frame[7]);
            Battery_VarRxReset(idx);
            return;
        }

        rx->reply_index  = hdr_reply_index;
        rx->expected_len = frame[7];
        rx->received_len = 0U;
        rx->crc_len      = 0U;
        rx->recv_start   = 1U;
        rx->next_seq     = 1U;
        LOG_DEBUG("[BAT%u] 0x55 rx start reply_index=0x%04lX len=%u",
                  (unsigned)(idx + 1U), (unsigned long)(rx->reply_index & 0xFFFFUL), (unsigned)rx->expected_len);
        return;
    }

    // 不是第一包数据 并且没有收到首帧 直接返回
    if (0U == rx->recv_start) {
        return;
    }

    //2. 校验序列号是否连续
    if (frame[1] != rx->next_seq) {
        LOG_WARN("[BAT%u] 0x55 seq err reply_index=0x%04lX expect=%u got=%u",
                 (unsigned)(idx + 1U), (unsigned long)(rx->reply_index & 0xFFFFUL),
                 (unsigned)rx->next_seq, (unsigned)frame[1]);
        Battery_VarRxReset(idx);
        return;
    }
    // 序列号加1
    rx->next_seq++;

    //3. 接收数据
    for (i = 2U; i <= 5U; i++) {
        if (rx->received_len < rx->expected_len) {
            rx->data[rx->received_len++] = frame[i];
        } else if (rx->crc_len < 2U) {
            rx->crc[rx->crc_len++] = frame[i];
        }
        if ((rx->received_len == rx->expected_len) && (2U == rx->crc_len)) {
            break;
        }
    }

    // 没有收到完整数据和CRC，直接返回
    if ((rx->received_len != rx->expected_len) || (2U != rx->crc_len)) {
        return;
    }

    //4. 校验CRC
    uint16_t crc_calc = Battery_ModbusCrc16(rx->data, rx->expected_len);
    uint16_t crc_rx   = (uint16_t)rx->crc[0] | ((uint16_t)rx->crc[1] << 8);

    if (crc_calc != crc_rx) {
        LOG_WARN("[BAT%u] 0x55 crc err reply_index=0x%04lX calc=0x%04X got=0x%04X",
                 (unsigned)(idx + 1U), (unsigned long)(rx->reply_index & 0xFFFFUL),
                 (unsigned)crc_calc, (unsigned)crc_rx);
        Battery_VarRxReset(idx);
        return;
    }

    //5. 解析数据
    Battery_OnVarValueDone(idx, rx->reply_index, rx->data, rx->expected_len);
    Battery_VarRxReset(idx);
}

/* -----------------------------------------------------------------------
 * CAN 帧分发入口（由 CanPort_Task 调用）
 * ----------------------------------------------------------------------- */
void BatteryComm_ParseCanFrame(uint8_t bat_idx, const uint8_t *rx_buf, uint8_t len)
{
    uint32_t can_id;

    if ((NULL == rx_buf) || (len < BAT_FRAME_LEN)) {
        return;
    }

    can_id = BAT_CAN_ID[bat_idx];

    LOG_DEBUG("[BAT%u] recv ID:0x%02lX DATA: %02X %02X %02X %02X %02X %02X %02X %02X",
              (unsigned)(bat_idx + 1U), (unsigned long)can_id,
              rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
              rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

    switch (rx_buf[0]) {
    case BAT_CMD_VERSION:
        Battery_ParseVersionReply(bat_idx, rx_buf, len);
        break;
    case BAT_CMD_REPORT_DATA:
        Battery_ParseDataReport(bat_idx, rx_buf, len);
        break;
    case BAT_CMD_QUERY_DATA:
        Battery_ParseVarReply(bat_idx, rx_buf, len);
        break;
    default:
        break;
    }
}
