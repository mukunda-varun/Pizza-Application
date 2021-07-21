/*
 * LHS_Pos_Sens.h
 *
 *  Created on: Jul 14, 2020
 *      Author: Admin
 */

#ifndef INC_LHS_POS_SENS_H_
#define INC_LHS_POS_SENS_H_


#define 	ADC_VAL_MIN_MM			383
#define 	MIN_VAL_MM				0
#define 	ADC_VAL_MAX_MM			2800
#define 	MAX_VAL_MM				15//15
#define 	MM_M_VAL				0.005917//0.005820
#define		MM_C_VAL				-2.113502//-1.976744

#define		ADC_12BIT_VAL			4095
#define 	MM_VAL					20.0
#define 	VOLTAGE_VAL				4.0
#define		OFFSET_DIST				7.1
//#define		CUTOFF_DIST				4.2					//Tested was getting 2.9mm
#define 	PIZZA_THICKNESS			3.0						//Pizza Thickness
#define		CUTOFF_DIST				(OFFSET_DIST - PIZZA_THICKNESS)
#define		ZERO_PWM				0.0
#define		PRESS_PWM				40.0
#define		PRESS_HOME_PWM			40.0

struct calib_M_C
{
	float adc_m;
	float adc_c;
}ADC_mm;

void LHS_Start_ADC(ADC_HandleTypeDef *hadc);
uint32_t LHS_Get_ADC(ADC_HandleTypeDef *hadc, int count);
uint8_t Auto_Calibrate(ADC_HandleTypeDef *hadc);
void Auto_Reverse_Calibrate(ADC_HandleTypeDef *hadc);
float LHS_ADC_TO_MM(uint32_t ADC_VAL);
float LHS_ADC_TO_VTG(uint32_t ADC_VAL);
void LHS_Stop_ADC(ADC_HandleTypeDef *hadc);
float readDistance(uint32_t adc);

#endif /* INC_LHS_POS_SENS_H_ */
