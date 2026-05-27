#ifndef __CT03A00_PORT_H__
#define __CT03A00_PORT_H__

#include "din.h"
#include "dout.h"

/* 
    CT03-A00 Orin 载板各 IO 端口定义
    参考 .\doc 下 PJ0050_CT03_SCH_A00_202601226A.pdf 文件
*/

/* 
	输入端口部分
*/
// PE2: 有线急停按键, EMERG_STOP
declare_din_pin(EMERG_STOP);
// PE3: 电池开关机, KEY_DET
declare_din_pin(KEY_DET);
// PC13: 电池插入检测, BAT_INSERT_DET
declare_din_pin(BAT_INSERT_DET);
// PB2: Orin 模块心跳检测, MODULE_HEART
declare_din_pin(MODULE_HEART);
// PE9: Orin 模块关机提前中断, MODULE_SHDN_INT
declare_din_pin(MODULE_SHDN_INT);
// PB13: 激活时，表示HV/MV电源故障, VIN_PWR_BAD_N_3V3
declare_din_pin(VIN_PWR_BAD_N_3V3);
// PA8: 模块固定检测, CVM_PRSNT
declare_din_pin(CVM_PRSNT);
// PA9: 模块复位检测, MODULE_RST_DET
declare_din_pin(MODULE_RST_DET);
// PC11: RK3588_PWRDN_DET
declare_din_pin(RK3588_PWRDN_DET);
// PD5: RK3288 模块插入检测, RK_INSERT_DET2
declare_din_pin(RK_INSERT_DET2);
// PD6: RK3288 模块插入检测, RK_INSERT_DET1
declare_din_pin(RK_INSERT_DET1);
// PD7: Orin 模块插入检测, ORIN_INSERT_DET2
declare_din_pin(ORIN_INSERT_DET2);
// PB3: Orin 模块插入检测, ORIN_INSERT_DET1
declare_din_pin(ORIN_INSERT_DET1);



/* 
	输出端口部分 
*/
// PE5: 扩展 IO 供电, VCC_5V_EXIO_EN
declare_dout_pin(VCC_5V_EXIO_EN);
// PA3: 关节预充使能, MOTOR_PCHG_EN
declare_dout_pin(MOTOR_PCHG_EN);
// PA4: 关节开关使能, MOTOR_CHG_EN
declare_dout_pin(MOTOR_CHG_EN);
// PA7: 充电使能, CHG_EN
declare_dout_pin(CHG_EN);
// PE7: Orin 模块关机控制, MODULE_BTN_3V3
declare_dout_pin(MODULE_BTN_3V3);
// PE11: 图传 5V USB EN, MCU_USB_UP
declare_dout_pin(MCU_USB_UP);
// PB10: MODULE_RST
declare_dout_pin(MODULE_RST);
// PB12: 外扩网口2电源开关, VCC_20V_EX2_EN
declare_dout_pin(VCC_20V_EX2_EN);
// PB14: 外扩网口1电源开关, VCC_20V_EX1_EN
declare_dout_pin(VCC_20V_EX1_EN);
// PB15: 图传模块电源开关, VCC_12V_TC_EN
declare_dout_pin(VCC_12V_TC_EN);
// PD9: 狗腿风扇电源开关, VCC_12V_GT_EN
declare_dout_pin(VCC_12V_GT_EN);
// PD10: 狗背风扇电源开关, VCC_12V_GB_EN
declare_dout_pin(VCC_12V_GB_EN);
// PD11: Orin 3.3V 供电, VCC_3V3_OR_EN
declare_dout_pin(VCC_3V3_OR_EN);
// PD13: RK 板电源开关, VCC_12V_RK_EN
declare_dout_pin(VCC_12V_RK_EN);
// PD14: RK 板电源开关, VCC_3V3_RK_EN
declare_dout_pin(VCC_3V3_RK_EN);
// PD15: RK 板电源开关, VCC_5V_OR_EN
declare_dout_pin(VCC_5V_OR_EN);
// PC6: Vbat-->20V@5A, VCC_24V_EX_ON
declare_dout_pin(VCC_24V_EX_ON);
// PC7: Vbat-->12V@6A, VCC_12V_A_ON
declare_dout_pin(VCC_12V_A_ON);
// PC8: Vbat-->18.4V@6A, VCC_18V_ON
declare_dout_pin(VCC_18V_ON);
// PC9: Vbat-->5V@8A, VCC_5V_A_EN
declare_dout_pin(VCC_5V_A_EN);
// PA11: MODULE_PWRON_N
declare_dout_pin(MODULE_PWRON_N);
// PA12: VCC_20V_EX3_EN
declare_dout_pin(VCC_20V_EX3_EN);
// PD2: USER_POW24V_EN
declare_dout_pin(USER_POW24V_EN);
// PD3: USER_POWER_EN
declare_dout_pin(USER_POWER_EN);
// PB7: HC32F460 心跳灯控制, HC_LED
declare_dout_pin(HC_LED);
// PC10: RK3588_RSTN
declare_dout_pin(RK3588_RSTN);
// PC12: RK3588_PWRON
declare_dout_pin(RK3588_PWRON);


void ct03a00_port_init(void);


#endif
