/*
 * step_motor.h
 *
 *  Created on: Sep 21, 2020
 *      Author: Admin
 */

#ifndef INC_STEP_MOTOR_H_
#define INC_STEP_MOTOR_H_

#include "main.h"

/**/

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define period           1000
#define FULL_STEP        1.8
#define HALF_STEP        0.9
#define ONE_BY_FOURTH    0.45
#define ONE_BY_EIGHT     0.225
#define Timer_clock      240000000
/*Variables For Stepper motor*/




/*Function Decaration*/
void StopAllMotors();
void Stepper_Disp_Rotate(unsigned int RPM, unsigned int Step, Motor_Direction Direction, uint8_t MotorSel);
void Stepper_Ejector_Rotate(unsigned int RPM, unsigned int Step, Motor_Direction Direction);
void Stepper_Ejector_Homing(unsigned int RPM, Motor_Direction Direction);
void Stepper_Ejector_End(unsigned int RPM, Motor_Direction Direction);
void Stepper_Kneader_Rotate(unsigned int RPM, unsigned int Step, Motor_Direction Direction);
void Stepper_Leadscrew_RotateTop(unsigned int RPM, Motor_Direction Direction);
void Stepper_Leadscrew_RotateBottom(unsigned int RPM, Motor_Direction Direction);
//void StopDispensingMotors(Dispensing_Stepper Motor);
void StopEjectorMotor(void);
void StopLeadscrewMotor(void);
/*Function Declaration*/

#endif /* INC_STEP_MOTOR_H_ */
