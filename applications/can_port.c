/**
 *******************************************************************************
 * @file  can_port.c
 * @brief CAN 应用适配层：FreeRTOS 队列、回调与调试输出
 *******************************************************************************
 */

#include "can_port.h"

#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "battery_comm.h"
#include "charger.h"
#include "can.h"
#include "log.h"

/* CAN 接收消息（中断 -> 队列 -> 任务） */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  buf[8];
} can_port_msg_t;

/* CAN 发送消息（业务 -> 队列 -> 任务） */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  buf[8];
} can_port_tx_msg_t;

#define CAN_PORT_LINE_LEN               96U
#define CAN_TX_QUEUE_DEPTH              16U

static QueueHandle_t s_can_rx_queue;
static QueueHandle_t s_can_tx_queue;

/* CAN 接收中断回调：仅入队，不做打印/解析 */
static void CanPort_RecvCallback(uint32_t id, uint8_t *buf, uint8_t len)
{
    BaseType_t     xHigherPriorityTaskWoken = pdFALSE;
    can_port_msg_t msg;
    uint8_t        i;

    msg.id  = id;
    msg.len = (len > 8U) ? 8U : len;

    for (i = 0U; i < msg.len; i++) {
        msg.buf[i] = buf[i];
    }
    for (; i < 8U; i++) {
        msg.buf[i] = 0U;
    }

    /* 中断里只做拷贝和入队，格式化/打印放到 CanPort_Task 里处理 */
    if (NULL != s_can_rx_queue) {
        (void)xQueueSendFromISR(s_can_rx_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* CAN 错误回调 */
static void CanPort_ErrorCallback(can_error_t err, const char *err_msg)
{
    stc_can_error_info_t info;

    (void)CAN_GetErrorInfo(CM_CAN, &info);
    // LOG_ERROR("CAN_ERR %u TEC=%u REC=%u %s",
    //             (unsigned int)err,
    //             (unsigned int)info.u8TxErrorCount,
    //             (unsigned int)info.u8RxErrorCount,
    //             (err_msg != NULL) ? err_msg : "");
}

/* 按 CAN ID 路由到对应协议模块 */
static void CanPort_Dispatch(const can_port_msg_t *msg)
{
    switch (msg->id) {
    case 0x40U:
        BatteryComm_ParseCanFrame(BAT1, msg->buf, msg->len);
        break;
    case 0x41U:
        BatteryComm_ParseCanFrame(BAT2, msg->buf, msg->len);
        break;
    case 0x44U:
        Charger_ParseCanFrame(msg->id, msg->buf, msg->len);
        break;
    default:
        break;
    }
}

/* 格式化 CAN 接收帧：整行一次 LOG_DEBUG，避免 DMA 串口输出交错 */
static int CanPort_FormatRxLine(const can_port_msg_t *msg, char *line, size_t line_size)
{
    static const char hex[] = "0123456789abcdef";
    int     pos;
    uint8_t i;

    if ((NULL == msg) || (NULL == line) || (0U == line_size)) {
        return -1;
    }

    pos = snprintf(line, line_size, "CAN_RX ID:0x%08lx LEN:%d DATA:",
                   (unsigned long)msg->id, (int)msg->len);
    if ((pos < 0) || ((size_t)pos >= line_size)) {
        line[line_size - 1U] = '\0';
        return -1;
    }

    for (i = 0U; i < msg->len; i++) {
        /* 每个字节追加 " xx"，并预留 "\r\n\0" */
        if (((size_t)pos + 6U) > line_size) {
            break;
        }

        line[pos++] = ' ';
        line[pos++] = hex[(msg->buf[i] >> 4U) & 0x0FU];
        line[pos++] = hex[msg->buf[i] & 0x0FU];
    }

    if (((size_t)pos + 3U) > line_size) {
        line[line_size - 1U] = '\0';
        return -1;
    }

    line[pos++] = '\r';
    line[pos++] = '\n';
    line[pos] = '\0';

    return pos;
}

/* CAN 应用初始化：队列 + 1M 波特率 + 回调注册 */
void CanPort_Init(void)
{
    s_can_rx_queue = xQueueCreate(32U, sizeof(can_port_msg_t));
    s_can_tx_queue = xQueueCreate(CAN_TX_QUEUE_DEPTH, sizeof(can_port_tx_msg_t));

    can_init(CAN_BAUDRATE_1M);
    can_set_recv_callback(CanPort_RecvCallback);
    can_set_error_callback(CanPort_ErrorCallback);
}

/* 业务层调用：将发送帧投入队列，由 CanPort_Task 统一发出 */
int CanPort_Send(uint32_t id, const uint8_t *buf, uint8_t len)
{
    can_port_tx_msg_t msg;
    uint8_t           i;

    if ((NULL == buf) || (len == 0U) || (len > 8U)) {
        return -1;
    }
    if (NULL == s_can_tx_queue) {
        return -1;
    }

    msg.id  = id;
    msg.len = len;
    for (i = 0U; i < len; i++) {
        msg.buf[i] = buf[i];
    }
    for (; i < 8U; i++) {
        msg.buf[i] = 0U;
    }

    return (pdTRUE == xQueueSend(s_can_tx_queue, &msg, pdMS_TO_TICKS(5U))) ? 0 : -1;
}

void CanPort_Task(void *param)
{
    can_port_msg_t    rx_msg;
    can_port_tx_msg_t tx_msg;
    char              line[CAN_PORT_LINE_LEN];

    (void)param;

    CanPort_Init();

    for (;;) {
        /* 等待接收帧，最多阻塞 1ms，保证 TX 队列能被及时处理 */
        if (pdTRUE == xQueueReceive(s_can_rx_queue, &rx_msg, pdMS_TO_TICKS(1U))) {
            /* 先打印原始帧数据（调试用） */
            // if (CanPort_FormatRxLine(&rx_msg, line, sizeof(line)) > 0) {
            //     LOG_DEBUG("%s", line);
            // }

            /* 按 CAN ID 路由到对应协议模块 */
            CanPort_Dispatch(&rx_msg);
        }

        /* 排干发送队列 */
        while (pdTRUE == xQueueReceive(s_can_tx_queue, &tx_msg, 0U)) {
            (void)can_send_std_frame(tx_msg.id, tx_msg.buf, tx_msg.len);
        }
    }
}
