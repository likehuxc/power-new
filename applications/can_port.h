/**
 *******************************************************************************
 * @file  can_port.h
 * @brief CAN 应用适配层：队列、回调与任务入口
 *******************************************************************************
 */

#ifndef CAN_PORT_H__
#define CAN_PORT_H__

#include <stdint.h>

void CanPort_Init(void);
void CanPort_Task(void *param);

/**
 * 将一帧标准帧数据放入发送队列，由 CanPort_Task 统一调度发出。
 * 可在任意任务上下文中调用，线程安全。
 * @return  0  入队成功
 *         -1  队列满或参数非法
 */
int CanPort_Send(uint32_t id, const uint8_t *buf, uint8_t len);

#endif /* CAN_PORT_H__ */
