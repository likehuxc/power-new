#include "ct02_port.h"

/*
    输入端口部分
*/
// PB2: 线充在位检测, CHG_ADPT_DET_IN, 高电平有效（硬件 4.7K 下拉）
def_din_pin(CHG_ADPT_DET_IN, GPIO_PORT_B, GPIO_PIN_02, 5, 1);
// PC13: 电池1在位检测, BAT_INSERT_DET_1, 低电平有效（硬件 10K 上拉）
def_din_pin(BAT_INSERT_DET_1, GPIO_PORT_C, GPIO_PIN_13, 5, 0);
// PE2: 急停开关, EMERG_STOP, 低电平有效
def_din_pin(EMERG_STOP, GPIO_PORT_E, GPIO_PIN_02, 5, 0);
// PE3: 电池1按键检测, KEY_INSERT_DET_1, 低电平有效
def_din_pin(KEY_INSERT_DET_1, GPIO_PORT_E, GPIO_PIN_03, 5, 0);
// PE4: 电池2在位检测, BAT_INSERT_DET_2, 低电平有效（硬件 10K 上拉）
def_din_pin(BAT_INSERT_DET_2, GPIO_PORT_E, GPIO_PIN_04, 5, 0);
// PE5: 电池2按键检测, KEY_INSERT_DET_2, 低电平有效
def_din_pin(KEY_INSERT_DET_2, GPIO_PORT_E, GPIO_PIN_05, 5, 0);
// PE6: 总开关按键, BAT_SW_USER, 低电平有效（内部上拉，无外部电阻）
def_din_pin(BAT_SW_USER, GPIO_PORT_E, GPIO_PIN_06, 5, 0);
// PE7: 桩充在位检测, CHG_PILE_DET_IN, 高电平有效（硬件 4.7K 下拉）
def_din_pin(CHG_PILE_DET_IN, GPIO_PORT_E, GPIO_PIN_07, 5, 1);

/*
    输出端口部分
*/
// PA4: 电机预充使能, MOTOR_PCHG_EN, 高电平有效
def_dout_pin(MOTOR_PCHG_EN, GPIO_PORT_A, GPIO_PIN_04, 1);
// PA5: 电机主供电使能, MOTOR_CHG_EN, 高电平有效
def_dout_pin(MOTOR_CHG_EN, GPIO_PORT_A, GPIO_PIN_05, 1);
// PA7: 充电/通信使能, CHARGER_EN, 高电平有效
def_dout_pin(CHARGER_EN, GPIO_PORT_A, GPIO_PIN_07, 1);
// PC7: 12V电源使能, PW_12V_EN, 高电平有效
def_dout_pin(PW_12V_EN, GPIO_PORT_C, GPIO_PIN_07, 1);
// PC8: 电机泄放, MOTOR_DISCHARGE_EN, 高电平有效
def_dout_pin(MOTOR_DISCHARGE_EN, GPIO_PORT_C, GPIO_PIN_08, 1);
// PE0: 系统心跳灯, HC_HEART_LED, 高电平有效
def_dout_pin(HC_HEART_LED, GPIO_PORT_E, GPIO_PIN_00, 1);


void ct02_port_init(void)
{
    /* 输入端口初始化 */
    din_init(&CHG_ADPT_DET_IN);
    din_init(&BAT_INSERT_DET_1);
    din_init(&EMERG_STOP);
    din_init(&KEY_INSERT_DET_1);
    din_init(&BAT_INSERT_DET_2);
    din_init(&KEY_INSERT_DET_2);
    din_init(&BAT_SW_USER);
    din_init(&CHG_PILE_DET_IN);

    /* 输出端口初始化 */
    dout_init(&MOTOR_PCHG_EN);
    dout_init(&MOTOR_CHG_EN);
    dout_init(&CHARGER_EN);
    dout_init(&PW_12V_EN);
    dout_init(&MOTOR_DISCHARGE_EN);
    dout_init(&HC_HEART_LED);
}
