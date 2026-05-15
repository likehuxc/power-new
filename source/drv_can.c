/**
 *******************************************************************************
 * @file  drv_can.c
 *******************************************************************************
 */

#include "drv_can.h"

#include <string.h>

#include "board.h"
#include "hc32_ll.h"

/* CAN 外设和 PB8/PB9 引脚复用配置 */
#define CAN_UNIT                        (CM_CAN)
#define CAN_PERIPH_CLK                  (FCG1_PERIPH_CAN)

#define CAN_TX_PORT                     (GPIO_PORT_B)
#define CAN_TX_PIN                      (GPIO_PIN_09)
#define CAN_TX_PIN_FUNC                 (GPIO_FUNC_50)

#define CAN_RX_PORT                     (GPIO_PORT_B)
#define CAN_RX_PIN                      (GPIO_PIN_08)
#define CAN_RX_PIN_FUNC                 (GPIO_FUNC_51)

/* 接收全部标准帧和扩展帧 */
#define CAN_FILTER_SEL                  (CAN_FILTER1)
#define CAN_FILTER_NUM                  (1U)
#define CAN_FILTER_ID                   (0UL)
#define CAN_FILTER_ID_MASK              (0x1FFFFFFFUL)
#define CAN_FILTER_ID_TYPE              (CAN_ID_STD_EXT)

/* 发送等待增加上限，避免异常时阻塞主循环 */
#define CAN_TX_WAIT_LOOP_MAX            (100000UL)

static int32_t CanWaitStatus(uint32_t u32Flag);

/* 初始化 CAN 外设 */
int32_t DrvCan_Init(void)
{
    int32_t i32Ret;
    stc_can_init_t stcCanInit;
    stc_can_filter_config_t astcFilter[CAN_FILTER_NUM] = {
        {CAN_FILTER_ID, CAN_FILTER_ID_MASK, CAN_FILTER_ID_TYPE},
    };

    Board_PeriphUnlock();

    GPIO_SetFunc(CAN_TX_PORT, CAN_TX_PIN, CAN_TX_PIN_FUNC);
    GPIO_SetFunc(CAN_RX_PORT, CAN_RX_PIN, CAN_RX_PIN_FUNC);

    (void)CAN_StructInit(&stcCanInit);
    stcCanInit.stcBitCfg.u32Prescaler = 2U;
    stcCanInit.stcBitCfg.u32TimeSeg1  = 6U;
    stcCanInit.stcBitCfg.u32TimeSeg2  = 2U;
    stcCanInit.stcBitCfg.u32SJW       = 2U;
    stcCanInit.pstcFilter             = astcFilter;
    stcCanInit.u16FilterSelect        = CAN_FILTER_SEL;
    stcCanInit.u8WorkMode             = CAN_WORK_MD_ELB;
    stcCanInit.u8SelfAck              = CAN_SELF_ACK_ENABLE;

    FCG_Fcg1PeriphClockCmd(CAN_PERIPH_CLK, ENABLE);
    i32Ret = CAN_Init(CAN_UNIT, &stcCanInit);
    if (LL_OK == i32Ret) {
        CAN_IntCmd(CAN_UNIT, CAN_INT_ALL, ENABLE);
    }

    Board_PeriphLock();

    return i32Ret;
}

/* 发送 CAN 帧 */
int32_t DrvCan_Send(const stc_drv_can_frame_t *pstcFrame)
{
    int32_t i32Ret;
    stc_can_tx_frame_t stcTxFrame;

    if ((NULL == pstcFrame) || (pstcFrame->u8DLC > DRV_CAN_DATA_SIZE_MAX)) {
        return LL_ERR_INVD_PARAM;
    }

    (void)memset(&stcTxFrame, 0, sizeof(stcTxFrame));
    stcTxFrame.u32ID = pstcFrame->u32ID;
    stcTxFrame.IDE   = pstcFrame->u8IDE;
    stcTxFrame.DLC   = pstcFrame->u8DLC;
    (void)memcpy(stcTxFrame.au8Data, pstcFrame->au8Data, pstcFrame->u8DLC);

    i32Ret = CAN_FillTxFrame(CAN_UNIT, CAN_TX_BUF_PTB, &stcTxFrame);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    CAN_StartTx(CAN_UNIT, CAN_TX_REQ_PTB);
    i32Ret = CanWaitStatus(CAN_FLAG_PTB_TX);
    if (LL_OK == i32Ret) {
        CAN_ClearStatus(CAN_UNIT, CAN_FLAG_PTB_TX);
    }

    return i32Ret;
}

/* 读取 CAN 帧 */
int32_t DrvCan_Read(stc_drv_can_frame_t *pstcFrame)
{
    int32_t i32Ret;
    stc_can_rx_frame_t stcRxFrame;

    if (NULL == pstcFrame) {
        return LL_ERR_INVD_PARAM;
    }

    i32Ret = CAN_GetRxFrame(CAN_UNIT, &stcRxFrame);
    if (LL_OK == i32Ret) {
        pstcFrame->u32ID    = stcRxFrame.u32ID;
        pstcFrame->u8IDE    = (uint8_t)stcRxFrame.IDE;
        pstcFrame->u8DLC    = (uint8_t)stcRxFrame.DLC;
        pstcFrame->u8SelfTx = (uint8_t)stcRxFrame.TX;
        (void)memcpy(pstcFrame->au8Data, stcRxFrame.au8Data, DRV_CAN_DATA_SIZE_MAX);
    }

    return i32Ret;
}

/* 等待 CAN 状态 */
static int32_t CanWaitStatus(uint32_t u32Flag)
{
    uint32_t u32Loop;

    for (u32Loop = 0UL; u32Loop < CAN_TX_WAIT_LOOP_MAX; u32Loop++) {
        if (SET == CAN_GetStatus(CAN_UNIT, u32Flag)) {
            return LL_OK;
        }
    }

    return LL_ERR_TIMEOUT;
}
