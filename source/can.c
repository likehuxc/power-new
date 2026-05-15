/**
 *******************************************************************************
 * @file  can.c
 *******************************************************************************
 */

#include "can.h"

#include <string.h>

#include "board.h"
#include "drv_can.h"
#include "hc32_ll.h"

#define CAN_LOOPBACK_PERIOD_MS          500UL  /* 循环发送周期 */
#define CAN_LOOPBACK_DLC                8U     /* 循环发送数据长度 */

#define CAN_TX_ID1                      0x1UL /* 循环发送ID1 */
#define CAN_TX_ID1_IDE                  0U
#define CAN_TX_ID2                      0x2UL /* 循环发送ID2 */
#define CAN_TX_ID2_IDE                  1U
#define CAN_TX_ID3                      0x3UL /* 循环发送ID3 */
#define CAN_TX_ID3_IDE                  1U

static stc_drv_can_frame_t m_stcCanTx1;
static stc_drv_can_frame_t m_stcCanTx2;
static stc_drv_can_frame_t m_stcCanTx3;
static int32_t             m_i32CanLastError = LL_OK;

static int32_t CanLoopback_Send(void);
static void CanLoopback_Rx(void);
static int32_t CanLoopback_Verify(const stc_drv_can_frame_t *pstcExpectFrame,
                                  const stc_drv_can_frame_t *pstcRxFrame);

int32_t Can_Init(void)
{
    return DrvCan_Init();
}

void Can_Task(void)
{
    static uint32_t u32LastTick;
    uint32_t        u32NowTick;

    u32NowTick = Board_GetTick();
    if ((u32NowTick - u32LastTick) >= CAN_LOOPBACK_PERIOD_MS) {
        u32LastTick = u32NowTick;
        m_i32CanLastError = CanLoopback_Send();
        if (LL_OK == m_i32CanLastError) {
            CanLoopback_Rx();
        }
    }
}

static int32_t CanLoopback_Send(void)
{
    uint8_t i;
    static uint8_t u8Data;
    int32_t i32Ret;

    for (i = 0U; i < CAN_LOOPBACK_DLC; i++) {
        m_stcCanTx1.au8Data[i] = u8Data++;
        m_stcCanTx2.au8Data[i] = u8Data++;
        m_stcCanTx3.au8Data[i] = u8Data++;
    }

    m_stcCanTx1.u32ID = CAN_TX_ID1;
    m_stcCanTx1.u8IDE = CAN_TX_ID1_IDE;
    m_stcCanTx1.u8DLC = CAN_LOOPBACK_DLC;
    i32Ret = DrvCan_Send(&m_stcCanTx1);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    m_stcCanTx2.u32ID = CAN_TX_ID2;
    m_stcCanTx2.u8IDE = CAN_TX_ID2_IDE;
    m_stcCanTx2.u8DLC = CAN_LOOPBACK_DLC;
    i32Ret = DrvCan_Send(&m_stcCanTx2);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    m_stcCanTx3.u32ID = CAN_TX_ID3;
    m_stcCanTx3.u8IDE = CAN_TX_ID3_IDE;
    m_stcCanTx3.u8DLC = CAN_LOOPBACK_DLC;
    return DrvCan_Send(&m_stcCanTx3);
}

static void CanLoopback_Rx(void)
{
    stc_drv_can_frame_t stcRxFrame;

    while (LL_OK == DrvCan_Read(&stcRxFrame)) {
        if (1U == stcRxFrame.u8SelfTx) {
            switch (stcRxFrame.u32ID) {
                case CAN_TX_ID1:
                    m_i32CanLastError = CanLoopback_Verify(&m_stcCanTx1, &stcRxFrame);
                    break;
                case CAN_TX_ID2:
                    m_i32CanLastError = CanLoopback_Verify(&m_stcCanTx2, &stcRxFrame);
                    break;
                case CAN_TX_ID3:
                    m_i32CanLastError = CanLoopback_Verify(&m_stcCanTx3, &stcRxFrame);
                    break;
                default:
                    m_i32CanLastError = LL_ERR;
                    break;
            }
        }
    }
}

static int32_t CanLoopback_Verify(const stc_drv_can_frame_t *pstcExpectFrame,
                                  const stc_drv_can_frame_t *pstcRxFrame)
{
    if ((pstcExpectFrame->u8DLC == pstcRxFrame->u8DLC) &&
        (0 == memcmp(pstcExpectFrame->au8Data, pstcRxFrame->au8Data, pstcRxFrame->u8DLC))) {
        return LL_OK;
    }

    return LL_ERR;
}
