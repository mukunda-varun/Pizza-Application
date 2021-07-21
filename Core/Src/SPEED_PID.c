/*
 * Speed_PID.c
 *
 *  Created on: 27-Jul-2020
 *      Author: svaru
 */
#include "Speed_PID.h"
/*Extern Variables*/
extern volatile float pre_error;
extern volatile int integral;
extern volatile float error;
extern volatile float derivative;
/*Extern Variables*/
UART_HandleTypeDef huart2;
float PIDcalc(float setpoint,float actual_position,float Kp, float Ki, float Kd)
{


	float pwm_val;
	float final_pwm_val;
//	char buff[300]="";
//	sprintf(buff, "Speed measured: %0.4lf \r\n", actual_position);
//	HAL_UART_Transmit(&huart2,(uint8_t *)buff, strlen(buff), 1000 );
	//CaculateP,I,D
	error = setpoint - actual_position;			//Proportional Value
	//In case of error too small then stop integration
	if(abs(error) > 1.0)
	{
		integral = integral + error;
	}
//	derivative= (error - pre_error);
	pwm_val = (Kp * abs(error)) + (Ki * abs(integral)) + (Kd * abs(derivative));
	final_pwm_val = (ceil(pwm_val *8) / 16);		//Round-off to nearest 0.125%
	//Saturation Filter
	if(final_pwm_val > PWM_MAX)
	{
		final_pwm_val= PWM_MAX;
	}
	else if(final_pwm_val < PWM_MIN)
	{
		final_pwm_val= PWM_MIN;
	}
//	sprintf(buff, "Proportional: %0.4lf Integral:%0.4lf Derivative: %0.4lf previous error:%0.4lf \r\n", error, integral, derivative,pre_error);
//	HAL_UART_Transmit(&huart2,(uint8_t *)buff, strlen(buff), 1000 );
	//Update error
	pre_error= error;
	return final_pwm_val;
}

