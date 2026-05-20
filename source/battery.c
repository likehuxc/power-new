/**
 *******************************************************************************
 * @file  battery.c
 * @brief 电池协议模块：CAN 命令发送、应答帧解析、周期查询任务
 *
 * 协议概述：
 *   - 所有帧为标准 CAN 帧，8 字节，byte7 = byte0..byte6 的异或校验
 *   - 下行（主机→电池）：byte0=0x00 为请求前缀，byte1=命令码，byte2=子索引
 *   - 上行（电池→主机）：byte0=命令码，byte1=子索引，byte2..5=大端数据
 *
 * 支持的命令：
 *   CMD 0x82 0x97 : 复位
 *   CMD 0x13      : 版本查询/应答
 *   CMD 0x82      : 数据查询/应答（满充容量、剩余容量、电压）
 *   CMD 0x00/0x55 : 不定长索引查询/应答（电池 SN，索引 0x5080）
 *******************************************************************************
 */

#include "battery.h"

#include "FreeRTOS.h"
#include "task.h"

#include "can.h"
#include "log.h"

/* 电池 CAN 标准帧 ID */
#define BATTERY_CAN_ID_RXD              0x41U
#define BATTERY_CAN_ID_TXD              0x41U

/* -----------------------------------------------------------------------
 * 命令与 payload 定义
 * 格式：byte0..byte6（共 7 字节），byte7 由 Battery_SendFrame() 自动计算 XOR
 * 复位
 * 版本查询
 * 满充容量查询
 * 剩余容量查询
 * 电压查询
 * 新增命令只需在此处添加一行数组定义即可，无需新建发送函数。
 * ----------------------------------------------------------------------- */
static const uint8_t BAT_CMD_RESET[7]             = { 0x82, 0x97, 0x00, 0x00, 0x00, 0x00, 0x01 };  /* 复位命令 */
static const uint8_t BAT_CMD_VERSION_QUERY[7]     = { 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00 };  /* 版本查询 */
static const uint8_t BAT_CMD_FULL_CAP_QUERY[7]    = { 0x00, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00 };  /* 满充容量查询 */
static const uint8_t BAT_CMD_REMAIN_CAP_QUERY[7]  = { 0x00, 0x82, 0x00, 0x00, 0x00, 0x01, 0x00 };  /* 剩余容量查询 */
static const uint8_t BAT_CMD_VOLT_QUERY[7]        = { 0x00, 0x82, 0x00, 0x00, 0x00, 0x02, 0x00 };  /* 电压查询 */
static const uint8_t BAT_CMD_CURRENT_QUERY[7]     = { 0x00, 0x82, 0x00, 0x00, 0x00, 0x0E, 0x00 };  /* 电流查询 */
static const uint8_t BAT_CMD_SN_QUERY[7]          = { 0x00, 0x55, 0x00, 0x00, 0x50, 0x80, 0x41 };  /* SN 不定长查询（索引 0x5080） */

/* 电池 SN：索引 0x5080，走 0x55 多帧不定长应答 */
#define BAT_SN_INDEX        0x5080U
#define BAT_SN_BUF_MAX      64U

static struct {
    uint8_t data[BAT_SN_BUF_MAX];  /* value[0..n-1] 拼接区 */
    uint8_t expect;                /* 帧0 byte7：数据长度 n（不含 CRC） */
    uint8_t count;                 /* 已收 value 字节数 */
    uint8_t crc[2];                /* 对端 CRC16，低字节在前 */
    uint8_t crc_len;                /* 已收 CRC 字节数 */
    uint8_t busy;                  /* 1=正在接收本次 SN */
} s_sn;


/* -----------------------------------------------------------------------
 * 公共发送 API
 *
 * 对外暴露简洁接口，调用方无需关心 payload 数组与 CAN ID。
 * 返回值：0=发送成功，-1=发送失败
 * ----------------------------------------------------------------------- */
static int Battery_SendFrame(uint32_t can_id, const uint8_t payload[7]);
static void Battery_SnRxReset(void);

/* 第三参数可选：发送前预处理（如 SN 需 SnRxReset） */
#define BATTERY_SEND_FUNC(name, cmd, ...) \
    int Battery_Send_##name(void)       \
    {                                   \
        __VA_ARGS__                     \
        Log_Printf("[BAT] send %s\r\n", #name); \
        return Battery_SendFrame(BATTERY_CAN_ID_TXD, (cmd)); \
    }

BATTERY_SEND_FUNC(GetVersion,        BAT_CMD_VERSION_QUERY)
BATTERY_SEND_FUNC(GetFullCapacity,   BAT_CMD_FULL_CAP_QUERY)
BATTERY_SEND_FUNC(GetRemainCapacity, BAT_CMD_REMAIN_CAP_QUERY)
BATTERY_SEND_FUNC(GetVoltage,        BAT_CMD_VOLT_QUERY)
BATTERY_SEND_FUNC(GetCurrent,        BAT_CMD_CURRENT_QUERY)
BATTERY_SEND_FUNC(GetSN,             BAT_CMD_SN_QUERY, Battery_SnRxReset();)
BATTERY_SEND_FUNC(Reset,             BAT_CMD_RESET)


/* -----------------------------------------------------------------------
 * Battery 任务
 *
 * 启动流程：
 *   1. 发送复位命令，等待电池初始化完成
 *   2. 进入主循环，轮询查询版本/容量/电压，每条间隔 1s
 *
 * 应答帧不在此任务处理，而是由 CanPort_Task 里 Battery_ParseCanFrame 解析。
 * ----------------------------------------------------------------------- */
void Battery_Task(void *param)
{
    int ret;

    (void)param;

    /* 第一步：发送复位命令，让电池进入工作状态 */
    // ret = Battery_Send_Reset();
    // if (0 == ret) {
    //     Log_Printf("[BAT] reset TX ok (ID 0x%02lX)\r\n",
    //                (unsigned long)BATTERY_CAN_ID_TXD);
    // } else {
    //     Log_Printf("[BAT] reset TX fail (ID 0x%02lX)\r\n",
    //                (unsigned long)BATTERY_CAN_ID_TXD);
    // }

    // /* 等待电池复位完成（电池收到复位后会上发一长串初始化数据） */
    // vTaskDelay(pdMS_TO_TICKS(3000));

    /* 第二步：轮询查询，每条命令间隔 3s，避免总线拥堵 测试发现2500ms也可以正常收到数据心跳数据*/
    for (;;) {
        // Battery_Send_GetSN();
        Battery_Send_GetVersion();
        vTaskDelay(pdMS_TO_TICKS(2700));
    }
}

/* -----------------------------------------------------------------------
 * 内部工具函数
 * ----------------------------------------------------------------------- */

/**
 * @brief  计算异或校验和
 * @param  buf  数据缓冲区
 * @param  len  参与校验的字节数
 * @return 逐字节异或结果
 */
static uint8_t Battery_CalcXorChecksum(const uint8_t *buf, uint8_t len)
{
    uint8_t cs = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

/**
 * @brief  从字节数组中读取大端 32 位无符号整数
 * @param  buf  指向 4 字节数据（高字节在前，对应协议 byte2..byte5）
 * @return 转换后的 uint32_t 值
 */
static uint32_t Battery_ReadU32BE(const uint8_t *buf)
{
    return (((uint32_t)buf[0]) << 24) |
           (((uint32_t)buf[1]) << 16) |
           (((uint32_t)buf[2]) <<  8) |
           (((uint32_t)buf[3]));
}

/**
 * @brief  版本字节转数字：ASCII '0'..'9'（0x30..0x39）减 0x30，否则原样使用
 */
static uint8_t Battery_DecVersionDigit(uint8_t raw)
{
    if (raw >= 0x30U) {
        return (uint8_t)(raw - 0x30U);
    }
    return raw;
}

/** Modbus CRC16：对 value[0..n-1] 校验，多项式 0xA001，初值 0xFFFF */
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

static void Battery_SnRxReset(void)
{
    s_sn.busy = s_sn.expect = s_sn.count = s_sn.crc_len = 0U;
}

/**
 * @brief  解析 0x55 多帧 SN 应答（索引 0x5080）
 * 帧0：byte2..5 为 4 字节 index（大端右对齐，0x5080 在 byte4/byte5），byte7=长度 n
 * 帧1+：byte2..5 拼 value，满 n 字节后接 Modbus CRC16 低、高（可跨帧）
 */
static void Battery_ParseSnReply(const uint8_t *f, uint8_t len)
{
    uint8_t  i;
    uint16_t idx;
    uint16_t crc_calc;
    uint16_t crc_rx;

    if ((len < 8U) || (0x55U != f[0])) {
        return;
    }

    /* 帧0：确认索引，取长度 n */
    if (0U == f[1]) {
        idx = ((uint16_t)f[4] << 8) | f[5];
        if ((BAT_SN_INDEX != idx) || (0U == f[7]) || (f[7] > BAT_SN_BUF_MAX)) {
            if ((BAT_SN_INDEX == idx) && (f[7] > BAT_SN_BUF_MAX)) {
                Log_Printf("[BAT] SN invalid len %u\r\n", (unsigned)f[7]);
            }
            Battery_SnRxReset();
            return;
        }
        s_sn.expect  = f[7];
        s_sn.count   = 0U;
        s_sn.crc_len = 0U;
        s_sn.busy    = 1U;
        Log_Printf("[BAT] SN rx start, len=%u\r\n", (unsigned)s_sn.expect);
        return;
    }

    if (0U == s_sn.busy) {
        return;
    }

    /* 帧1+：先拼 value，再收 CRC；收齐后立即 break，避免继续消费 byte4/5 */
    for (i = 2U; i <= 5U; i++) {
        if (s_sn.count < s_sn.expect) {
            s_sn.data[s_sn.count++] = f[i];
        } else if (s_sn.crc_len < 2U) {
            s_sn.crc[s_sn.crc_len++] = f[i];
        }
        if ((s_sn.count == s_sn.expect) && (2U == s_sn.crc_len)) {
            break;
        }
    }

    if ((s_sn.count != s_sn.expect) || (2U != s_sn.crc_len)) {
        return;
    }

    /* 收齐：Modbus CRC16 校验（crc[0]低字节，crc[1]高字节） */
    crc_calc = Battery_ModbusCrc16(s_sn.data, s_sn.expect);
    crc_rx   = (uint16_t)s_sn.crc[0] | ((uint16_t)s_sn.crc[1] << 8);
    if (crc_calc != crc_rx) {
        Log_Printf("[BAT] SN crc err (calc=0x%04X got=0x%04X)\r\n",
                    (unsigned)crc_calc, (unsigned)crc_rx);
        Battery_SnRxReset();
        return;
    }

    Log_Printf("[BAT] SN (%u bytes): ", (unsigned)s_sn.expect);
    for (i = 0U; i < s_sn.expect; i++) {
        Log_Printf("%c", (char)s_sn.data[i]);
    }
    Log_Printf("\r\n----------------------------------------\r\n");
    Battery_SnRxReset();
}

/**
 * @brief  通用 CAN 帧发送
 * @param  can_id   CAN 标准帧 ID
 * @param  payload  7 字节有效载荷（byte0..byte6）
 * @return 0=成功，-1=失败
 * @note   byte7 自动填充为 byte0..byte6 的异或校验和
 */
static int Battery_SendFrame(uint32_t can_id, const uint8_t payload[7])
{
    uint8_t tx_buf[8];
    uint8_t i;

    for (i = 0U; i < 7U; i++) {
        tx_buf[i] = payload[i];
    }
    tx_buf[7] = Battery_CalcXorChecksum(tx_buf, 7U);
   Log_Printf("[BAT] tran ID:0x%02lX DATA: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
              can_id, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
              tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]);
    return can_send_std_frame(can_id, tx_buf, 8U);
}

/* -----------------------------------------------------------------------
 * 应答帧解析
 * ----------------------------------------------------------------------- */

/**
 * @brief  解析 0x13 版本应答帧
 *
 * 帧格式（rx_buf = [0x13, b1, b2, b3, b4, ...]）：
 *   Byte[0] = 0x13（命令标识）
 *   Byte[1..4] 字节存在两种解释，这里同时打印两种结果便于现场对照：
 *     解释一（节点单体版本）：byte1=主, byte2=子, byte3=修订, byte4=固定 0x00
 *     解释二（多电池汇总版）：byte1=电池1 主, byte2=电池1 子,
 *                              byte3=电池2 主, byte4=电池2 子
 *   Byte[5..6] = 保留
 *   Byte[7]    = XOR 校验
 *
 * 数据清洗：原始字节若 ≥ 0x30 视为 ASCII 数字（'0'..'9'）减 0x30，
 * 否则按纯数字使用，由 Battery_DecVersionDigit() 完成。
 */
static void Battery_ParseVersionReply(const uint8_t *rx_buf, uint8_t len)
{
    uint8_t cs_calc;

    /* 长度保护：必须满 8 字节才能做 byte7 校验 */
    if (len < 8U) {
        Log_Printf("[BAT] version reply too short (%u)\r\n", (unsigned)len);
        return;
    }

    /* 校验验证 */
    cs_calc = Battery_CalcXorChecksum(rx_buf, 7U);
    if (cs_calc != rx_buf[7]) {
        Log_Printf("[BAT] version checksum err (calc=0x%02X got=0x%02X)\r\n",
                    (unsigned)cs_calc, (unsigned)rx_buf[7]);
        return;
    }

    /* 原始字节提取（msg.buf = [0x13, 0x31, 0x30, 0x31, 0x00, ...]） */
    /* byte1：解释一-主版本   / 解释二-电池1 主 */
    /* byte2：解释一-子版本   / 解释二-电池1 子 */
    /* byte3：解释一-修订版本 / 解释二-电池2 主 */
    /* byte4：解释一-固定0x00 / 解释二-电池2 子 */
    uint8_t v_b1_ma = Battery_DecVersionDigit(rx_buf[1]);
    uint8_t v_b1_mi = Battery_DecVersionDigit(rx_buf[2]);
    uint8_t v_b1_re = Battery_DecVersionDigit(rx_buf[3]);
    uint8_t v_b2_mi = Battery_DecVersionDigit(rx_buf[4]);

    Log_Printf("[VER] (单版本): V%u.%u.%u\r\n",
                (unsigned)v_b1_ma, (unsigned)v_b1_mi, (unsigned)v_b1_re);
    Log_Printf("[VER] (多电池): 1:V%u.%u, 2:V%u.%u\r\n",
                (unsigned)v_b1_ma, (unsigned)v_b1_mi,
                (unsigned)v_b1_re, (unsigned)v_b2_mi);
    Log_Printf("----------------------------------------\r\n");
}

/**
 * @brief  解析 0x82 数据应答帧
 *
 * 帧格式：
 *   Byte[0] = 0x82（命令标识）
 *   Byte[1] = sub_index（子索引）
 *   Byte[2..5] = 大端 32 位数据
 *   Byte[6] = 保留
 *   Byte[7] = XOR 校验
 *
 * 子索引定义：
 *   0x00 = 满充容量（mAh，uint32_t）
 *   0x01 = 剩余容量（mAh，uint32_t）
 *   0x02 = 电池电压（mV，int32_t，打印为 V）
 *   0x0E = 电池电流（mA，int32_t，充电>0 放电≤0）
 */
static void Battery_ParseDataReply(const uint8_t *rx_buf, uint8_t len)
{
    uint8_t  cs_calc;
    uint8_t  sub_index;
    uint32_t raw_u32;
    int32_t  raw_i32;

    /* 长度保护 */
    if (len < 8U) {
        Log_Printf("[BAT] data reply too short (%u)\r\n", (unsigned)len);
        return;
    }

    /* 校验验证 */
    cs_calc = Battery_CalcXorChecksum(rx_buf, 7U);
    if (cs_calc != rx_buf[7]) {
        Log_Printf("[BAT] data checksum err (calc=0x%02X got=0x%02X)\r\n",
                    (unsigned)cs_calc, (unsigned)rx_buf[7]);
        return;
    }


    /* 提取子索引和数据块 */
    sub_index = rx_buf[1];
    raw_u32   = Battery_ReadU32BE(&rx_buf[2]);  /* byte2..byte5 大端 */

    switch (sub_index) {
    case 0x00U:  /* 满充容量 */
        Log_Printf("[BAT] full capacity: %lu mAh\r\n", (unsigned long)raw_u32);
        break;

    case 0x01U:  /* 剩余容量 */
        Log_Printf("[BAT] remain capacity: %lu mAh\r\n", (unsigned long)raw_u32);
        break;

    case 0x02U:  /* 电压（mV 转 V，整数格式避免浮点依赖） */
        raw_i32 = (int32_t)raw_u32;
        Log_Printf("[BAT] voltage: %ld.%03ld V\r\n",
                    (long)(raw_i32 / 1000),
                    (long)(raw_i32 >= 0 ? raw_i32 % 1000 : -(raw_i32 % 1000)));
        break;

    case 0x0EU:  /* 电池电流（mA，int32，充电>0 放电≤0） */
        raw_i32 = (int32_t)raw_u32;
        Log_Printf("[BAT] current: %ld mA\r\n", (long)raw_i32);
        break;

    default:  /* 未知子索引，打印原始值供调试 */
        break;
    }
}

/* -----------------------------------------------------------------------
 * CAN 帧分发入口
 *
 * 由 CanPort_Task() 在任务上下文中调用，根据 byte0 命令码分派到对应解析函数。
 * 这样 can_port.c 不需要了解电池协议细节，所有协议逻辑集中在 battery.c。
 * ----------------------------------------------------------------------- */
void Battery_ParseCanFrame(uint32_t can_id, const uint8_t *rx_buf, uint8_t len)
{
    (void)can_id;  /* 暂不过滤 CAN ID，后续可按需添加 */

    if ((NULL == rx_buf) || (len < 1U)) {
        return;
    }

    Log_Printf("[BAT] recv ID:0x%02lX DATA: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               (unsigned long)can_id,
               rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

    switch (rx_buf[0]) {
    case 0x13U:  /* 版本应答 */
        Battery_ParseVersionReply(rx_buf, len);
        break;

    case 0x82U:  /* 数据应答（容量/电压） */
        Battery_ParseDataReply(rx_buf, len);
        break;

    case 0x55U:  /* SN 多帧应答 */
        Battery_ParseSnReply(rx_buf, len);
        break;

    default:
        /* 未识别的命令码，静默忽略 */
        break;
    }
}
