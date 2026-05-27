#include "ct03a00_port.h"

/*
    输入端口部分
*/
// PE2: 有线急停按键, EMERG_STOP, 低电平有效
def_din_pin(EMERG_STOP, GPIO_PORT_E, GPIO_PIN_02, 5, 0);
// PE3: 电池开关机, KEY_DET
def_din_pin(KEY_DET, GPIO_PORT_E, GPIO_PIN_03, 5, 0);
// PC13: 电池插入检测, BAT_INSERT_DET
def_din_pin(BAT_INSERT_DET, GPIO_PORT_C, GPIO_PIN_13, 5, 0);
// PB2: Orin 模块心跳检测, MODULE_HEART
def_din_pin(MODULE_HEART, GPIO_PORT_B, GPIO_PIN_02, 5, 0);
// PE9: Orin 模块关机提前中断, MODULE_SHDN_INT
def_din_pin(MODULE_SHDN_INT, GPIO_PORT_E, GPIO_PIN_09, 5, 0);
// PB13: 激活时，表示HV/MV电源故障, VIN_PWR_BAD_N_3V3
def_din_pin(VIN_PWR_BAD_N_3V3, GPIO_PORT_B, GPIO_PIN_13, 5, 0);
// PA8: 模块固定检测, CVM_PRSNT
def_din_pin(CVM_PRSNT, GPIO_PORT_A, GPIO_PIN_08, 5, 0);
// PA9: 模块复位检测, MODULE_RST_DET
def_din_pin(MODULE_RST_DET, GPIO_PORT_A, GPIO_PIN_09, 5, 0);
// PC11: RK3588_PWRDN_DET
def_din_pin(RK3588_PWRDN_DET, GPIO_PORT_C, GPIO_PIN_11, 5, 0);
// PD5: RK3288 模块插入检测, RK_INSERT_DET2
def_din_pin(RK_INSERT_DET2, GPIO_PORT_D, GPIO_PIN_05, 5, 0);
// PD6: RK3288 模块插入检测, RK_INSERT_DET1
def_din_pin(RK_INSERT_DET1, GPIO_PORT_D, GPIO_PIN_06, 5, 0);
// PD7: Orin 模块插入检测, ORIN_INSERT_DET2
def_din_pin(ORIN_INSERT_DET2, GPIO_PORT_D, GPIO_PIN_07, 5, 0);
// PB3: Orin 模块插入检测, ORIN_INSERT_DET1
def_din_pin(ORIN_INSERT_DET1, GPIO_PORT_B, GPIO_PIN_03, 5, 0);


/* 
	输出端口部分 
*/
// PE5: 扩展 IO 供电, VCC_5V_EXIO_EN
def_dout_pin(VCC_5V_EXIO_EN, GPIO_PORT_E, GPIO_PIN_05, 1);
// PA3: 关节预充使能, MOTOR_PCHG_EN
def_dout_pin(MOTOR_PCHG_EN, GPIO_PORT_A, GPIO_PIN_03, 1);
// PA4: 关节开关使能, MOTOR_CHG_EN
def_dout_pin(MOTOR_CHG_EN, GPIO_PORT_A, GPIO_PIN_04, 1);
// PA7: 充电使能, CHG_EN
def_dout_pin(CHG_EN, GPIO_PORT_A, GPIO_PIN_07, 1);
// PE7: Orin 模块关机控制, MODULE_BTN_3V3
def_dout_pin(MODULE_BTN_3V3, GPIO_PORT_E, GPIO_PIN_07, 0);
// PE11: 图传 5V USB EN, MCU_USB_UP
def_dout_pin(MCU_USB_UP, GPIO_PORT_E, GPIO_PIN_11, 1);
// PB10: MODULE_RST
def_dout_pin(MODULE_RST, GPIO_PORT_B, GPIO_PIN_10, 0);
// PB12: 外扩网口2电源开关, VCC_20V_EX2_EN
def_dout_pin(VCC_20V_EX2_EN, GPIO_PORT_B, GPIO_PIN_12, 1);
// PB14: 外扩网口1电源开关, VCC_20V_EX1_EN
def_dout_pin(VCC_20V_EX1_EN, GPIO_PORT_B, GPIO_PIN_14, 1);
// PB15: 图传模块电源开关, VCC_12V_TC_EN
def_dout_pin(VCC_12V_TC_EN, GPIO_PORT_B, GPIO_PIN_15, 1);
// PD9: 狗腿风扇电源开关, VCC_12V_GT_EN
def_dout_pin(VCC_12V_GT_EN, GPIO_PORT_D, GPIO_PIN_09, 1);
// PD10: 狗背风扇电源开关, VCC_12V_GB_EN
def_dout_pin(VCC_12V_GB_EN, GPIO_PORT_D, GPIO_PIN_10, 1);
// PD11: Orin 3.3V 供电, VCC_3V3_OR_EN
def_dout_pin(VCC_3V3_OR_EN, GPIO_PORT_D, GPIO_PIN_11, 1);
// PD13: RK 板电源开关, VCC_12V_RK_EN
def_dout_pin(VCC_12V_RK_EN, GPIO_PORT_D, GPIO_PIN_13, 1);
// PD14: RK 板电源开关, VCC_3V3_RK_EN
def_dout_pin(VCC_3V3_RK_EN, GPIO_PORT_D, GPIO_PIN_14, 1);
// PD15: RK 板电源开关, VCC_5V_OR_EN
def_dout_pin(VCC_5V_OR_EN, GPIO_PORT_D, GPIO_PIN_15, 1);
// PC6: Vbat-->20V@5A, VCC_24V_EX_ON
def_dout_pin(VCC_24V_EX_ON, GPIO_PORT_C, GPIO_PIN_06, 1);
// PC7: Vbat-->12V@6A, VCC_12V_A_ON
def_dout_pin(VCC_12V_A_ON, GPIO_PORT_C, GPIO_PIN_07, 1);
// PC8: Vbat-->18.4V@6A, Orin 模块主电源, VCC_18V_ON
def_dout_pin(VCC_18V_ON, GPIO_PORT_C, GPIO_PIN_08, 1);
// PC9: Vbat-->5V@8A, VCC_5V_A_EN
def_dout_pin(VCC_5V_A_EN, GPIO_PORT_C, GPIO_PIN_09, 1);
// PA11: MODULE_PWRON_N
def_dout_pin(MODULE_PWRON_N, GPIO_PORT_A, GPIO_PIN_11, 0);
// PA12: VCC_20V_EX3_EN
def_dout_pin(VCC_20V_EX3_EN, GPIO_PORT_A, GPIO_PIN_12, 1);
// PD2: USER_POW24V_EN
def_dout_pin(USER_POW24V_EN, GPIO_PORT_D, GPIO_PIN_02, 0);
// PD3: USER_POWER_EN
def_dout_pin(USER_POWER_EN, GPIO_PORT_D, GPIO_PIN_03, 0);
// PB7: HC32F460 心跳灯控制, HC_LED
def_dout_pin(HC_LED, GPIO_PORT_B, GPIO_PIN_07, 1);
// PC10: RK3588_RSTN
def_dout_pin(RK3588_RSTN, GPIO_PORT_C, GPIO_PIN_10, 1);
// PC12: RK3588_PWRON
def_dout_pin(RK3588_PWRON, GPIO_PORT_C, GPIO_PIN_12, 1);


void ct03a00_port_init(void)
{
    // 输入端口初始化
    din_init(&EMERG_STOP);
    din_init(&KEY_DET);
    din_init(&BAT_INSERT_DET);
    din_init(&MODULE_HEART);
    din_init(&MODULE_SHDN_INT);
    din_init(&VIN_PWR_BAD_N_3V3);
    din_init(&CVM_PRSNT);
    din_init(&MODULE_RST_DET);
    din_init(&RK3588_PWRDN_DET);
    din_init(&RK_INSERT_DET2);
    din_init(&RK_INSERT_DET1);
    din_init(&ORIN_INSERT_DET2);
    din_init(&ORIN_INSERT_DET1);

    // 输出端口初始化
    dout_init(&VCC_5V_EXIO_EN);
    dout_init(&MOTOR_PCHG_EN);
    dout_init(&MOTOR_CHG_EN);
    dout_init(&CHG_EN);
    dout_init(&MODULE_BTN_3V3);
    dout_init(&MCU_USB_UP);
    dout_init(&MODULE_RST);
    dout_init(&VCC_20V_EX2_EN);
    dout_init(&VCC_20V_EX1_EN);
    dout_init(&VCC_12V_TC_EN);
    dout_init(&VCC_12V_GT_EN);
    dout_init(&VCC_12V_GB_EN);
    dout_init(&VCC_3V3_OR_EN);
    dout_init(&VCC_12V_RK_EN);
    dout_init(&VCC_3V3_RK_EN);
    dout_init(&VCC_5V_OR_EN);
    dout_init(&VCC_24V_EX_ON);
    dout_init(&VCC_18V_ON);
    dout_init(&VCC_12V_A_ON);
    dout_init(&VCC_5V_A_EN);
    dout_init(&MODULE_PWRON_N);
    dout_init(&VCC_20V_EX3_EN);
    dout_init(&USER_POW24V_EN);
    dout_init(&USER_POWER_EN);
    dout_init(&HC_LED);
    dout_init(&RK3588_RSTN);
    dout_init(&RK3588_PWRON);

    // 默认输出设置
    dout_set_inactive(&USER_POWER_EN);
    dout_set_inactive(&USER_POW24V_EN);
    
}
