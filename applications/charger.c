/**
 *******************************************************************************
 * @file  charger.c
 * @brief 充电器协议模块（CAN ID 0x44）
 *
 * 协议待定，当前为桩实现。
 *******************************************************************************
 */

#include "charger.h"

#include <stddef.h>
#include <stdio.h>

#include "log.h"

#define CHARGER_LOG_LINE_LEN  80U

void Charger_ParseCanFrame(uint32_t can_id, const uint8_t *rx_buf, uint8_t len)
{
    char    line[CHARGER_LOG_LINE_LEN];
    int     pos;
    uint8_t i;

    if ((NULL == rx_buf) || (len < 1U)) {
        return;
    }

    /* TODO: 按充电器协议解析 */
    pos = snprintf(line, sizeof(line), "[CHG] recv ID:0x%02lX LEN:%u DATA:",
                   (unsigned long)can_id, (unsigned)len);
    if ((pos < 0) || (pos >= (int)sizeof(line))) {
        return;
    }

    for (i = 0U; i < len; i++) {
        if ((pos + 4) >= (int)sizeof(line)) {
            break;
        }
        pos += snprintf(&line[pos], sizeof(line) - (size_t)pos, " %02X", (unsigned)rx_buf[i]);
    }

    LOG_INFO("%s", line);
}
