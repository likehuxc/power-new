#ifndef __CAN_H__
#define __CAN_H__

#include "hc32_ll.h"

typedef enum {
    CAN_BAUDRATE_1M = 0,
    CAN_BAUDRATE_500K,
    CAN_BAUDRATE_250K,
    CAN_BAUDRATE_200K,
    CAN_BAUDRATE_125K,
    CAN_BAUDRATE_100K,
} can_baudrate_t;

typedef enum {
    CAN_ERROR_NONE,
    CAN_ERROR_BIT,
    CAN_ERROR_FORM,
    CAN_ERROR_STUFF,
    CAN_ERROR_ACK,
    CAN_ERROR_CRC,
    CAN_ERROR_OTHER,
    CAN_ERROR_BUS_OFF,
    CAN_ERROR_RX_OVERRUN,
} can_error_t;

void can_init(can_baudrate_t baudrate);
int  can_send_std_frame(uint32_t id, uint8_t* buf, uint8_t len);
int  can_send_ext_frame(uint32_t id, uint8_t* buf, uint8_t len);
void can_set_recv_callback(void (*recv)(uint32_t id, uint8_t* buf, uint8_t len));
void can_set_error_callback(void (*cb)(can_error_t err, const char* err_msg));
void can_set_send_callback(void (*cb)(void));

#endif
