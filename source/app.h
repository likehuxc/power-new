/**
 *******************************************************************************
 * @file  app.h
 *******************************************************************************
 */

#ifndef APP_H__
#define APP_H__

void App_Init(void);       /* 外设与 CAN 队列初始化 */
void App_StartTasks(void); /* 创建 FreeRTOS 任务，需在 vTaskStartScheduler 前调用 */

#endif /* APP_H__ */
