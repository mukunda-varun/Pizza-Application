/*
 * ADC_READ.c
 *
 *  Created on: Sep 24, 2020
 *      Author: Madhu
 */

#include "main.h"
#include "cmsis_os.h"
#include "ADC_READ.h"
uint32_t Linear_actuator_Pot_Raw_data,Depth_Sensor_Raw_Data;
uint32_t Adc_channel_reading_for_multiple_inputs(ADC_HandleTypeDef hadc,uint8_t inputs)
{
	uint8_t idx;
	uint32_t temp=0,temp2=0;
	for(idx=0;idx<50;idx++)
	{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc,10);
	temp+= HAL_ADC_GetValue(&hadc);
	HAL_ADC_PollForConversion(&hadc, 10);
	temp2 += HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop (&hadc);
	}
	temp/=50;
	Depth_Sensor_Raw_Data=temp2/50;
    return temp;
}
