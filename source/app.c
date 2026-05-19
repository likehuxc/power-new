/**
 *******************************************************************************
 * @file  app.c
 * @brief 应用层：FreeRTOS 任务创建，CAN 队列与回调（参考 d5w_pmu）
 *******************************************************************************
 */

#include "app.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "can.h"
#include "led.h"
#include "uart.h"

/* CAN 接收消息（中断 -> 队列 -> 任务） */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  buf[8];
} app_can_msg_t;

static QueueHandle_t s_can_rx_queue;

/* CAN 接收中断回调：仅入队，不做打印/解析 */
static void CanRecvCallback(uint32_t id, uint8_t *buf, uint8_t len)
{
    BaseType_t     xHigherPriorityTaskWoken = pdFALSE;
    app_can_msg_t  msg;
    uint8_t        i;

    msg.id  = id;
    msg.len = (len > 8U) ? 8U : len;

    for (i = 0U; i < msg.len; i++) {
        msg.buf[i] = buf[i];
    }
    for (; i < 8U; i++) {
        msg.buf[i] = 0U;
    }

    if (NULL != s_can_rx_queue) {
        (void)xQueueSendFromISR(s_can_rx_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* CAN 错误回调 */
static void CanErrorCallback(can_error_t err, const char *err_msg)
{
    (void)err;
    (void)err_msg;
    Uart_Printf("CAN_ERR\r\n");
}

/* CAN 应用初始化：队列 + 1M 波特率 + 回调注册 */
static void CanApp_Init(void)
{
    s_can_rx_queue = xQueueCreate(32, sizeof(app_can_msg_t));

    can_init(CAN_BAUDRATE_1M);
    can_set_recv_callback(CanRecvCallback);
    can_set_error_callback(CanErrorCallback);
}

/* LED 任务：周期调用 Led_Task，内部用 Board_GetTick() 做 1s 闪烁 */
static void LedThread(void *param)
{
    (void)param;

    for (;;) {
        Led_Task();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* USART1 任务：阻塞读接收队列并回显 */
static void Uart1Thread(void *param)
{
    (void)param;

    for (;;) {
        Uart1_Task();
    }
}

/* USART4 任务：阻塞读接收队列并回显 */
static void Uart4Thread(void *param)
{
    (void)param;

    for (;;) {
        Uart4_Task();
    }
}

/* CAN 任务：阻塞读队列，通过 USART1 打印收到的帧（不解析协议） */
static void CanThread(void *param)
{
    app_can_msg_t msg;

    (void)param;

    for (;;) {
        if (pdTRUE == xQueueReceive(s_can_rx_queue, &msg, portMAX_DELAY)) {
            Uart_Printf("CAN_RX ID:0x%08lx LEN:%u DATA:%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
                        (unsigned long)msg.id,
                        (unsigned int)msg.len,
                        (unsigned int)msg.buf[0], (unsigned int)msg.buf[1],
                        (unsigned int)msg.buf[2], (unsigned int)msg.buf[3],
                        (unsigned int)msg.buf[4], (unsigned int)msg.buf[5],
                        (unsigned int)msg.buf[6], (unsigned int)msg.buf[7]);
        }
    }
}

/* 外设与应用模块初始化（在启动调度器之前调用） */
void App_Init(void)
{
    /* 与 d5 一致，便于 FreeRTOS 中断优先级分组 */
    NVIC_SetPriorityGrouping(3U);

    Led_Init();
    Uart_Init();
    CanApp_Init();
}

/* 创建 LED / USART1 / USART4 / CAN 任务 */
void App_StartTasks(void)
{
    (void)xTaskCreate(LedThread, "led", 128, NULL, configMAX_PRIORITIES - 10, NULL);
    (void)xTaskCreate(Uart1Thread, "u1", 256, NULL, configMAX_PRIORITIES - 9, NULL);
//    (void)xTaskCreate(Uart4Thread, "u4", 256, NULL, configMAX_PRIORITIES - 8, NULL);
    (void)xTaskCreate(CanThread, "can", 256, NULL, configMAX_PRIORITIES - 7, NULL);
}
