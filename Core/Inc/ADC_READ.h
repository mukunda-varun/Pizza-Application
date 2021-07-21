/*
 * ADC_READ.h
 *
 *  Created on: Sep 24, 2020
 *      Author: Madhu
 */

#ifndef INC_ADC_READ_H_
#define INC_ADC_READ_H_
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

typedef enum
{
	SPARE_MOTOR_LSENSE,
	PRESS_MOTOR_LSENSE,
	TOGGLING_MOTOR_LSENSE,
	KNEAD_MOTOR_RSENSE,
	SPARE_MOTOR_RSENSE,
	PRESS_MOTOR_RSENSE,
	TOGGLING_MOTOR_RSENSE,
	KNEAD_MOTOR_LSENSE,
}Adc1_inputs_data_t;
typedef enum
{
	LINEAR_POSITION_SENSOR,
	SHARP_SENSOR,
}Adc2_inputs_data_t;
//#define 	ADC_VAL_MIN_MM			600//383
#define 	MIN_VAL_MM				0
//#define 	ADC_VAL_MAX_MM			4095//3612
//#define 	MAX_VAL_MM				20
#define 	MM_M_VAL				0.005917//0.005820
#define		MM_C_VAL				-2.113502//-1.976744
#define ADC1_TOTAL_INPUTCHANNELS   8
#define ADC2_TOTAL_INPUTCHANNELS   2
uint32_t Adc_channel_reading_for_multiple_inputs(ADC_HandleTypeDef hadc,uint8_t inputs);


#endif /* INC_ADC_READ_H_ */
