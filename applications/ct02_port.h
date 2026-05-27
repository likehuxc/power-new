#ifndef __CT02_PORT_H__
#define __CT02_PORT_H__

#include "din.h"
#include "dout.h"

/*
    CT02 电源板各 IO 端口定义
*/

/*
    输入端口部分
*/
// PB2: 线充在位检测, CHG_ADPT_DET_IN, 高电平有效
declare_din_pin(CHG_ADPT_DET_IN);
// PC13: 电池1在位检测, BAT_INSERT_DET_1, 低电平有效
declare_din_pin(BAT_INSERT_DET_1);
// PE2: 急停开关, EMERG_STOP, 低电平有效
declare_din_pin(EMERG_STOP);
// PE3: 电池1按键检测, KEY_INSERT_DET_1, 低电平有效
declare_din_pin(KEY_INSERT_DET_1);
// PE4: 电池2在位检测, BAT_INSERT_DET_2, 低电平有效
declare_din_pin(BAT_INSERT_DET_2);
// PE5: 电池2按键检测, KEY_INSERT_DET_2, 低电平有效
declare_din_pin(KEY_INSERT_DET_2);
// PE6: 总开关按键, BAT_SW_USER, 低电平有效
declare_din_pin(BAT_SW_USER);
// PE7: 桩充在位检测, CHG_PILE_DET_IN, 高电平有效
declare_din_pin(CHG_PILE_DET_IN);

/*
    输出端口部分
*/
// PA4: 电机预充使能, MOTOR_PCHG_EN, 高电平有效
declare_dout_pin(MOTOR_PCHG_EN);
// PA5: 电机主供电使能, MOTOR_CHG_EN, 高电平有效
declare_dout_pin(MOTOR_CHG_EN);
// PA7: 充电/通信使能, CHARGER_EN, 高电平有效
declare_dout_pin(CHARGER_EN);
// PC7: 12V电源使能, PW_12V_EN, 高电平有效
declare_dout_pin(PW_12V_EN);
// PC8: 电机泄放, MOTOR_DISCHARGE_EN, 高电平有效
declare_dout_pin(MOTOR_DISCHARGE_EN);
// PE0: 系统心跳灯, HC_HEART_LED, 高电平有效
declare_dout_pin(HC_HEART_LED);


void ct02_port_init(void);

#endif
