/**
 *******************************************************************************
 * @file  can_port.c
 * @brief CAN 应用适配层：FreeRTOS 队列、回调与调试输出
 *******************************************************************************
 */

#include "can_port.h"

#include <stdint.h>

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

    (void)param;

    CanPort_Init();

    for (;;) {
        /* 等待接收帧，最多阻塞 1ms，保证 TX 队列能被及时处理 */
        if (pdTRUE == xQueueReceive(s_can_rx_queue, &rx_msg, pdMS_TO_TICKS(1U))) {
            /* 先打印原始帧数据（调试用） */
            // LOG_DEBUG("CAN_RX ID:0x%08lx LEN:%d DATA: %02X %02X %02X %02X %02X %02X %02X %02X",
            //           (unsigned long)rx_msg.id, (int)rx_msg.len,
            //           rx_msg.buf[0], rx_msg.buf[1], rx_msg.buf[2], rx_msg.buf[3],
            //           rx_msg.buf[4], rx_msg.buf[5], rx_msg.buf[6], rx_msg.buf[7]);

            /* 按 CAN ID 路由到对应协议模块 */
            CanPort_Dispatch(&rx_msg);
        }

        /* 排干发送队列 */
        while (pdTRUE == xQueueReceive(s_can_tx_queue, &tx_msg, 0U)) {
            (void)can_send_std_frame(tx_msg.id, tx_msg.buf, tx_msg.len);
            LOG_DEBUG("CAN_TX ID:0x%08lx LEN:%d DATA: %02X %02X %02X %02X %02X %02X %02X %02X",
                      (unsigned long)tx_msg.id, (int)tx_msg.len,
                      tx_msg.buf[0], tx_msg.buf[1], tx_msg.buf[2], tx_msg.buf[3],
                      tx_msg.buf[4], tx_msg.buf[5], tx_msg.buf[6], tx_msg.buf[7]);
        }
    }
}
