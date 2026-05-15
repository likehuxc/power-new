/**
 *******************************************************************************
 * @file  app.c
 *******************************************************************************
 */

#include <stdbool.h>

#include "app.h"

#include "board.h"
#include "drv_led.h"
#include "drv_uart_dma.h"
#include "hc32_ll.h"

#define HEART_LED_BLINK_PERIOD_MS 1000UL
#define HEARTBEAT_MSG             "hello 111\r\n"

static uint8_t m_au8HeartbeatMsg[] = HEARTBEAT_MSG;

static void Heartbeat_Process(void);
static void UartFrame_Process(void);

static void Heartbeat_Process(void)
{
    static uint32_t u32LastTick;
    uint32_t        u32NowTick;

    u32NowTick = Board_GetTick();
    if ((u32NowTick - u32LastTick) >= HEART_LED_BLINK_PERIOD_MS) {
        u32LastTick = u32NowTick;
        DrvLed_Toggle();
        (void)DrvUartDma_Send(m_au8HeartbeatMsg,
                              (uint16_t)(ARRAY_SZ(m_au8HeartbeatMsg) - 1U));
    }
}

static void UartFrame_Process(void)
{
    uint8_t  au8Frame[DRV_UART_DMA_FRAME_LEN_MAX];
    uint16_t u16Len;

    if (true == DrvUartDma_ReadFrame(au8Frame, sizeof(au8Frame), &u16Len)) {
        /* au8Frame[0 .. u16Len-1] 为本帧数据，后续处理业务逻辑 */
    }
}

int32_t App_Init(void)
{
    int32_t i32Ret;

    DrvLed_Init();

    i32Ret = DrvUartDma_Init();
    return i32Ret;
}

void App_Process(void)
{
    Heartbeat_Process();
    UartFrame_Process();
}
