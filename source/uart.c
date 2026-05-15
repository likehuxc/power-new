/**
 *******************************************************************************
 * @file  uart.c
 *******************************************************************************
 */

#include "uart.h"

#include <stdbool.h>

#include "board.h"
#include "drv_uart_dma.h"
#include "hc32_ll.h"

#define UART_HEARTBEAT_PERIOD_MS        1000UL
#define UART_HEARTBEAT_MSG              "hello 111\r\n"

static uint8_t m_au8HeartbeatMsg[] = UART_HEARTBEAT_MSG;

static void UartHeartbeat_Process(void);
static void UartFrame_Process(void);

int32_t Uart_Init(void)
{
    return DrvUartDma_Init();
}

void Uart_Task(void)
{
    UartHeartbeat_Process();
    UartFrame_Process();
}

static void UartHeartbeat_Process(void)
{
    static uint32_t u32LastTick;
    uint32_t        u32NowTick;

    // u32NowTick = Board_GetTick();
    // if ((u32NowTick - u32LastTick) >= UART_HEARTBEAT_PERIOD_MS) {
    //     u32LastTick = u32NowTick;
    //     (void)DrvUartDma_Send(m_au8HeartbeatMsg,
    //                           (uint16_t)(ARRAY_SZ(m_au8HeartbeatMsg) - 1U));
    // }
}

static void UartFrame_Process(void)
{
    uint8_t  au8Frame[DRV_UART_DMA_FRAME_LEN_MAX];
    uint16_t u16Len;

    if (true == DrvUartDma_ReadFrame(au8Frame, sizeof(au8Frame), &u16Len)) {
        /* au8Frame[0 .. u16Len-1] 为本帧数据，后续可在这里接入业务解析 */
        (void)DrvUartDma_Send(au8Frame, u16Len);
    }
}
