#include "can.h"

static void (*recv_cb)(uint32_t id, uint8_t* buf, uint8_t len) = 0;
static void (*error_cb)(can_error_t err, const char* err_msg) = 0;
static void (*send_cb)(void) = 0;

/* 解析 CAN_FLAG_ERR_INT：可能是 ACK 失败、TEC/REC 越警告阈值、Bus-Off 等 */
static void can_handle_error_status(uint32_t status)
{
    stc_can_error_info_t err;

    if ((status & CAN_FLAG_RX_OVERRUN) != 0U) {
        if (error_cb) {
            error_cb(CAN_ERROR_RX_OVERRUN, "Rx Overrun");
        }
    }

    if ((status & CAN_FLAG_BUS_OFF) != 0U) {
        CAN_ExitLocalReset(CM_CAN);
        if (error_cb) {
            error_cb(CAN_ERROR_BUS_OFF, "Bus Off");
        }
        return;
    }

    (void)CAN_GetErrorInfo(CM_CAN, &err);

    if ((status & CAN_FLAG_BUS_ERR) != 0U) {
        switch (err.u8ErrorType) {
            case CAN_ERR_BIT:   if (error_cb) error_cb(CAN_ERROR_BIT, "Bit Error"); break;
            case CAN_ERR_FORM:  if (error_cb) error_cb(CAN_ERROR_FORM, "Form Error"); break;
            case CAN_ERR_STUFF: if (error_cb) error_cb(CAN_ERROR_STUFF, "Stuff Error"); break;
            case CAN_ERR_ACK:   if (error_cb) error_cb(CAN_ERROR_ACK, "ACK Error (no node ACK?)"); break;
            case CAN_ERR_CRC:   if (error_cb) error_cb(CAN_ERROR_CRC, "CRC Error"); break;
            case CAN_ERR_OTHER: if (error_cb) error_cb(CAN_ERROR_OTHER, "Other Error"); break;
            default:            if (error_cb) error_cb(CAN_ERROR_OTHER, "Bus err, type unknown"); break;
        }
    } else if ((status & CAN_FLAG_TEC_REC_WARN) != 0U) {
        if (error_cb) {
            error_cb(CAN_ERROR_OTHER, "TEC/REC warn threshold");
        }
    } else if ((status & CAN_FLAG_ERR_INT) != 0U) {
        /* 仅 EIF：多为发送无应答导致 TEC 变化，KOER 可能已为 0 */
        if (error_cb) {
            error_cb(CAN_ERROR_ACK, "ERR_INT (check TEC/ACK/baud/ID)");
        }
    }
}

// CAN 中断回调
static void can_irq_callback(void)
{
    uint32_t status;

    status = CAN_GetStatusValue(CM_CAN);

    /* 先处理再清除：原先先 Clear 会导致 GetErrorInfo 全 0 */
    if ((status & CAN_FLAG_RX) != 0U) {
        stc_can_rx_frame_t frame;
        if (CAN_GetRxFrame(CM_CAN, &frame) == LL_OK) {
            if (recv_cb && (frame.RTR == 0)) {
                /* DLC 是位域，先读到局部变量；经典 CAN 下 DLC 即字节数(0~8) */
                uint8_t rx_len = (uint8_t)frame.DLC;
                if (rx_len > 8U) {
                    rx_len = 8U;
                }
                recv_cb(frame.u32ID, frame.au8Data, rx_len);
            }
        }
    }

    if ((status & CAN_FLAG_PTB_TX) != 0U) {
        if (send_cb) {
            send_cb();
        }
    }

    if ((status & (CAN_FLAG_ERR_INT | CAN_FLAG_BUS_ERR | CAN_FLAG_BUS_OFF |
                   CAN_FLAG_RX_OVERRUN | CAN_FLAG_TEC_REC_WARN)) != 0U) {
        can_handle_error_status(status);
    }

    if (status != 0U) {
        CAN_ClearStatus(CM_CAN, status);
    }
}

// CAN 初始化
void can_init(can_baudrate_t baudrate)
{
    /*
        CAN GPIO 配置
    */
    // HC_CAN_RX: PB8 <---> Func51 (CAN_RxD)
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_51);
    // HC_CAN_TX: PB9 <---> Func50 (CAN_TxD)
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_50);

    /*
        CAN 时钟/波特率配置  
    */
    // 使能 CAN 时钟
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN, ENABLE);
    stc_can_init_t can;
    CAN_StructInit(&can);

#define PRESC   (can.stcBitCfg.u32Prescaler)
#define SEG_1   (can.stcBitCfg.u32TimeSeg1)
#define SEG_2   (can.stcBitCfg.u32TimeSeg2)
#define SJW     (can.stcBitCfg.u32SJW)

    // 波特率计算: baudrate = XTAL(12MHz) / [PRESC * (SEG_1 + SEG_2)]
    switch(baudrate) {                                                              // Sample point
        case CAN_BAUDRATE_1M:   PRESC = 1; SJW = 1; SEG_1 = 11; SEG_2 = 1; break;   // 1M, seg2>=sjw
        case CAN_BAUDRATE_500K: PRESC = 1; SJW = 3; SEG_1 = 21; SEG_2 = 3; break;   // 80.0%
        case CAN_BAUDRATE_250K: PRESC = 2; SJW = 2; SEG_1 = 21; SEG_2 = 3; break;   // 88.0%
        case CAN_BAUDRATE_200K: PRESC = 2; SJW = 2; SEG_1 = 26; SEG_2 = 4; break;   // 87.1%
        case CAN_BAUDRATE_125K: PRESC = 2; SJW = 2; SEG_1 = 42; SEG_2 = 6; break;   // 87.8%
        case CAN_BAUDRATE_100K: PRESC = 4; SJW = 1; SEG_1 = 26; SEG_2 = 4; break;   // 87.1%
        default: break;
    }
    // CAN 过滤配置
    can.pstcFilter             = 0;
    can.u16FilterSelect        = CAN_FILTER1;
    can.u8WorkMode             = CAN_WORK_MD_NORMAL;
    CAN_Init(CM_CAN, &can);

#undef PRESC
#undef SEG_1
#undef SEG_2
#undef SJW

    /*
        CAN 中断配置
    */
    CAN_IntCmd(CM_CAN, CAN_INT_ALL, DISABLE);
    CAN_IntCmd(CM_CAN, CAN_INT_PTB_TX | CAN_INT_RX | CAN_INT_ERR_INT, ENABLE);

    stc_irq_signin_config_t irq;
    irq.enIntSrc    = INT_SRC_CAN_INT;
    irq.enIRQn      = INT010_IRQn;   /* 避开 USART1 占用的 INT000~INT004 */
    irq.pfnCallback = &can_irq_callback;
    INTC_IrqSignIn(&irq);

    NVIC_ClearPendingIRQ(irq.enIRQn);
    NVIC_SetPriority(irq.enIRQn, DDL_IRQ_PRIO_01);
    NVIC_EnableIRQ(irq.enIRQn);
}

// CAN 发送帧
static int __send_frame(uint32_t id, uint8_t ext_id, uint8_t* buf, uint8_t len)
{
    stc_can_tx_frame_t frame;
    uint8_t  i;
    uint32_t ms;
	
	if(CM_CAN->CFG_STAT & CAN_CFG_STAT_RESET) {
		CM_CAN->CFG_STAT = 0;
	}
	
    if(len > 8) len = 8;

    frame.u32ID   = id;
    frame.u32Ctrl = 0;
    frame.DLC = len;
    frame.IDE = ext_id;

    for(i = 0; i < len; i ++) {
        frame.au8Data[i] = buf[i];
    }
	uint32_t retry = 0;
    while(CAN_FillTxFrame(CM_CAN, CAN_TX_BUF_PTB, &frame) != LL_OK) {
        if(++ retry >= 10000) return -1;
    }
    CAN_StartTx(CM_CAN, CAN_TX_REQ_PTB);
	
    return 0;
}

// CAN 发送标准帧
int can_send_std_frame(uint32_t id, uint8_t* buf, uint8_t len)
{
    return __send_frame(id, 0, buf, len);
}

// CAN 发送扩展帧
int can_send_ext_frame(uint32_t id, uint8_t* buf, uint8_t len)
{
    return __send_frame(id, 1, buf, len);
}

// 设置接收回调
void can_set_recv_callback(void (*cb)(uint32_t id, uint8_t* buf, uint8_t len))
{
    recv_cb = cb;
}

// 设置错误回调
void can_set_error_callback(void (*cb)(can_error_t err, const char* err_msg))
{
    error_cb = cb;
}

// 设置发送回调
void can_set_send_callback(void (*cb)(void))
{
    send_cb = cb;
}
