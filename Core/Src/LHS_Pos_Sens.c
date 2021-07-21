/*
 * LHS_Pos_Sens.c
 *
 *  Created on: Jul 14, 2020
 *      Author: Admin
 */
#include "main.h"
struct calib_M_C ADC_MM;
extern uint8_t press_stop;

UART_HandleTypeDef huart2;

/*	Brief: 		Sets the ADON bit to enable ADC and starts ADC for Polling
 * 	@Params:	hadc -> ADC to be used
 * 	Return: 	Nothing
 * 	*/
void LHS_Start_ADC(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
//	HAL_ADC_Start_IT(hadc);
}
/*	@Brief: 	Gets the ADC value from the channel and calculates the average for the number of counts
 * 	@Params: 	hadc -> ADC to be used
 * 				count-> Number of Samples to be considered for Averaging the ADC
 * 	@Return: 	The Average ADC value
 * 	*/
uint32_t LHS_Get_ADC(ADC_HandleTypeDef *hadc, int count)
{
	uint32_t retVal,adc_sum = 0;
	for(int i=0; i<count; i++)
	{
		adc_sum += HAL_ADC_GetValue(hadc);
	}
	retVal = adc_sum / count;
	return retVal;
}
/*	@Brief: 	Calculates the Calibrated m and c value for better accuracy
 * 	@Params: 	hadc -> ADC to be used
 * 	@Return: 	Nothing
 * 	NOTE: For calculation of m and c value, y values are the raw value(in mm) and x values are ADC counts
 * 	*/
uint8_t Auto_Calibrate(ADC_HandleTypeDef *hadc)
{
	uint32_t adc_val_0mm;
//	char buff[100]="", buff2[100]="";
	HAL_ADC_Stop_IT(hadc);
//	HAL_UART_Transmit(&huart2, (uint8_t *)"Calibrating for Accuracy.....!\r\n", sizeof("Calibrating for Accuracy.....!\r\n"), 100);
	/*ADC Poll*/
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	/*ADC Poll*/
	adc_val_0mm = LHS_Get_ADC(hadc, 100);
//	sprintf(buff2,"Calibration is done at: %ld \r\n", adc_val_0mm);
//	HAL_UART_Transmit(&huart2, (uint8_t *)buff2, sizeof(buff2), 100);
	/* Formula for m = ((y2-y1)/(x2-x1)) */
	ADC_mm.adc_m = ((float)(MAX_VAL_MM - (float)0)/((float)ADC_VAL_MAX_MM - (float)adc_val_0mm));
	/* Formula for c = ((y1x2-y2x1)/(x2-x1))*/
	ADC_mm.adc_c = ((((float)ADC_VAL_MAX_MM * (float)0)-((float)MAX_VAL_MM * (float)adc_val_0mm)) / ((float)ADC_VAL_MAX_MM - (float)adc_val_0mm));

//	sprintf(buff,"Calibrated Values are: \r\n"
//			"M = %f,  C = %f\r\n",ADC_mm.adc_m,ADC_mm.adc_c);
//	HAL_UART_Transmit(&huart2, (uint8_t *)buff, sizeof(buff), 100);
//	HAL_Delay(2000);
//	if((ADC_mm.adc_m != 0) && (ADC_mm.adc_c != 0))
		return 1;
//	else
//		return 0;
}

/*	@Brief: 	Calculates the Calibrated m and c value for better accuracy
 * 	@Params: 	hadc -> ADC to be used
 * 	@Return: 	Nothing
 * 	NOTE: For calculation of m and c value, y values are the raw value(in mm) and x values are ADC counts
 * 	*/
void Auto_Reverse_Calibrate(ADC_HandleTypeDef *hadc)
{
	uint32_t adc_val_0mm;
//	char buff[100]="";
//	HAL_UART_Transmit(&huart2, (uint8_t *)"Calibrating for Accuracy.....!\r\n", sizeof("Calibrating for Accuracy.....!\r\n"), 100);
	adc_val_0mm = LHS_Get_ADC(hadc, 4);
	/* Formula for m = ((y2-y1)/(x2-x1)) */
	ADC_mm.adc_m = ((float)(MAX_VAL_MM - (float)MIN_VAL_MM)/((float)ADC_VAL_MIN_MM - (float)adc_val_0mm));
	/* Formula for c = ((y2x1-y1x2)/(x1-x2))*/
	ADC_mm.adc_c = ((((float)MIN_VAL_MM * (float)ADC_VAL_MIN_MM)-((float)MAX_VAL_MM * (float)adc_val_0mm)) / ((float)ADC_VAL_MIN_MM - (float)adc_val_0mm));

//	sprintf(buff,"Calibrated Values are: \r\n"
//			"M = %f,  C = %f\r\n",ADC_mm.adc_m,ADC_mm.adc_c);
//	HAL_UART_Transmit(&huart2, (uint8_t *)buff, sizeof(buff), 100);
}
float readDistance(uint32_t adc)
{
	static uint32_t final_adc = 0;
	static double Distance=0.0;
	final_adc = adc / 4;
	Distance = LHS_ADC_TO_MM(final_adc);
	return Distance;
	if(Distance >= CUTOFF_DIST)
	{
		press_stop = 1;
	}
}
/*	@Brief: Converts the ADC value to actual value in mm usinig calibrated m and c values
 * 	@Params: 	ADC_VAL -> ADC Value
 * 	@Return: The Actual value in mm
 * 	NOTE: For calculation of m and c value, y values are the raw value(in mm) and x values are ADC counts*/
float LHS_ADC_TO_MM(uint32_t ADC_VAL)
{
	/*Formula used for converting is y=mx+c*/
	return ((float)ADC_VAL*(float)ADC_mm.adc_m+(float)(ADC_mm.adc_c));
//	return ((float)ADC_VAL*(float)MM_M_VAL+(float)(MM_C_VAL));
//	return (((((float)ADC_VAL - (float)ADC_VAL_OFFSET_VAL) / (float)ADC_12BIT_VAL)) * (float)MM_VAL);
}
/*	@Brief: 	Converts the ADC value to actual value in V
 * 	@Params: 	ADC_VAL -> ADC Value
 * 	@Return: 	The Actual value in V
 * 	(Not Used and Not accurate)*/
/*float LHS_ADC_TO_VTG(uint32_t ADC_VAL)
{
	return ((((float)ADC_VAL - (float)ADC_VAL_OFFSET_VAL)/ (float)ADC_12BIT_VAL) * (float)VOLTAGE_VAL);
}*/
/*	@Brief: 	Stops the ADC by disabling the ADON bit
 * 	@Params: 	hadc -> ADC to be used
 * 	@Return: 	The Average ADC value
 * 	*/
void LHS_Stop_ADC(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Stop(hadc);
}
