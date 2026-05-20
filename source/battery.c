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
 *******************************************************************************
 */

#include "battery.h"

#include "FreeRTOS.h"
#include "task.h"

#include "can.h"
#include "log.h"

/* 电池 CAN 标准帧 ID */
#define BATTERY_CAN_ID_STD              0x41U

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
static const uint8_t BAT_CMD_REMAIN_CAP_QUERY[7]  = { 0x00, 0x82, 0x01, 0x00, 0x00, 0x00, 0x00 };  /* 剩余容量查询 */
static const uint8_t BAT_CMD_VOLT_QUERY[7]        = { 0x00, 0x82, 0x02, 0x00, 0x00, 0x00, 0x00 };  /* 电压查询 */


/* -----------------------------------------------------------------------
 * 公共查询 API
 *
 * 对外暴露简洁接口，调用方无需关心 payload 数组与 CAN ID。
 * 返回值：0=发送成功，-1=发送失败
 * ----------------------------------------------------------------------- */
static int Battery_SendFrame(uint32_t can_id, const uint8_t payload[7]);

#define BATTERY_QUERY_FUNC(name, cmd) \
    int Battery_Query##name(void)     \
    {                                 \
        Log_Printf("[BAT] query %s\r\n", #name); \
        return Battery_SendFrame(BATTERY_CAN_ID_STD, (cmd)); \
    }

BATTERY_QUERY_FUNC(Version,        BAT_CMD_VERSION_QUERY)
BATTERY_QUERY_FUNC(FullCapacity,   BAT_CMD_FULL_CAP_QUERY)
BATTERY_QUERY_FUNC(RemainCapacity, BAT_CMD_REMAIN_CAP_QUERY)
BATTERY_QUERY_FUNC(Voltage,        BAT_CMD_VOLT_QUERY)


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
     ret = Battery_SendFrame(BATTERY_CAN_ID_STD, BAT_CMD_RESET);
     if (0 == ret) {
         Log_Printf("[BAT] reset TX ok (ID 0x%02lX)\r\n",
                     (unsigned long)BATTERY_CAN_ID_STD);
     } else {
         Log_Printf("[BAT] reset TX fail (ID 0x%02lX)\r\n",
                     (unsigned long)BATTERY_CAN_ID_STD);
    }

    /* 等待电池复位完成（电池收到复位后会上发一长串初始化数据） */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 第二步：轮询查询，每条命令间隔 2s，避免总线拥堵 */
    for (;;) {
        Battery_QueryVersion();
        vTaskDelay(pdMS_TO_TICKS(2000));
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

    default:  /* 未知子索引，打印原始值供调试 */
        Log_Printf("[BAT] data sub=0x%02X raw=0x%08lX\r\n",
                    (unsigned)sub_index, (unsigned long)raw_u32);
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

    switch (rx_buf[0]) {
    case 0x13U:  /* 版本应答 */
        Battery_ParseVersionReply(rx_buf, len);
        break;

    case 0x82U:  /* 数据应答（容量/电压） */
        Battery_ParseDataReply(rx_buf, len);
        break;

    default:
        /* 未识别的命令码，静默忽略 */
        break;
    }
}
