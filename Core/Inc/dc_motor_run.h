/*
 * dc_motor_run.h
 *
 *  Created on: Sep 21, 2020
 *      Author: Admin
 */

#ifndef INC_DC_MOTOR_RUN_H_
#define INC_DC_MOTOR_RUN_H_

#include "main.h"
//extern DC_Motor DC_Motor_Sel;
extern Motor_Direction DC_Motor_Dir;

void PWM_Channels_Init(void);
void PWM_Channels_Stop(void);
//void DC_Set_PWM(float pwm_val,uint8_t Direction, DC_Motor Motor);
void Press_DC_Set_PWM(float pwm_val,uint8_t Direction);
void Toggling_DC_Set_PWM(float pwm_val,uint8_t Direction);
void Blade_DC_Set_PWM(float pwm_val,uint8_t Direction);
//void DC_Motor_Stop(DC_Motor Motor_no);
void Press_DC_Motor_Stop(void);
void Toggling_DC_Motor_Stop(void);
void Blade_DC_Motor_Stop(void);
void Agitator_MTR_Run(float pwm, uint8_t direction);
void Agitator_MTR_Stop(void);

#endif /* INC_DC_MOTOR_RUN_H_ */
