/**
 *******************************************************************************
 * @file  log.h
 * @brief 日志模块：Log_Printf 写入 StreamBuffer，UartLog_Task 统一发送
 *******************************************************************************
 */

#ifndef LOG_H__
#define LOG_H__

#include <stdint.h>

int32_t Log_Init(void);
void UartLog_Task(void *param);
void Log_Printf(const char *fmt, ...);

#endif /* LOG_H__ */
