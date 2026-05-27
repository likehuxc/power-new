/**
 *******************************************************************************
 * @file  power_manage.h
 * @brief ADC 采样与电源监测量（滤波后电压/电流 ADC 值）
 *******************************************************************************
 */

#ifndef POWER_MANAGE_H__
#define POWER_MANAGE_H__

void PowerManage_Init(void);
void PowerManage_Task(void *pvParameters);

/* AI_MOTOR_BUS PA1/CH1 */
extern float g_motor_bus_voltage;
/* AI_CHG_ADPT PA6/CH6 */
extern float g_chg_adpt_voltage;
/* IBAT_OUT_2 PB0/CH8 */
extern float g_ibat_out_2;
/* AI_CHG_PILE PB1/CH9 */
extern float g_chg_pile_voltage;
/* AI_VBAT_IN_1 PC2/CH12 */
extern float g_vbat_in_1_voltage;
/* AI_VBAT_IN_2 PC3/CH13 */
extern float g_vbat_in_2_voltage;
/* IBAT_OUT_1 PC4/CH14 */
extern float g_ibat_out_1;
/* AI_VBAT_IN PC5/CH15 */
extern float g_vbat_in_voltage;

#endif /* POWER_MANAGE_H__ */
