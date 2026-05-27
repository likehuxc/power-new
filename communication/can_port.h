/**
 *******************************************************************************
 * @file  can_port.h
 * @brief CAN 应用适配层：队列、回调与任务入口
 *******************************************************************************
 */

#ifndef CAN_PORT_H__
#define CAN_PORT_H__

void CanPort_Init(void);
void CanPort_Task(void *param);

#endif /* CAN_PORT_H__ */
