/**
 *******************************************************************************
 * @file  log.h
 * @brief 日志模块：Log_Printf 写入 StreamBuffer，UartLog_Task 统一发送
 *******************************************************************************
 */

#ifndef LOG_H__
#define LOG_H__

#include <stdint.h>

#define LOG_INFO(fmt, ...) Log_Printf("[INFO] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) Log_Printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) Log_Printf("[WARN] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) Log_Printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)

int32_t Log_Init(void);
void UartLog_Task(void *param);
void Log_Printf(const char *fmt, ...);

#endif /* LOG_H__ */
