/*
 * dc_motor_run.c
 *
 *  Created on: Sep 21, 2020
 *      Author: Admin
 */
#include "dc_motor_run.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim3;




/* @brief: Initialize the PWM Channels
 * @return: NOTHING*/
void PWM_Channels_Init(void)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);		//Toggling Motor RPWM
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);		//Toggling Motor LPWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);		//Kneading Blade Motor RPWM
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);		//Kneading Blade MOtor LPWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);		//Press Motor RPWM
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);		//Press Motor LPWM
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);		//Agitator Motor PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);		//Spare Motor
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);		//Spare Motor
//	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);		//Spare Motor PWM


	TIM4->CCR1 = 0;
//	TIM4->CCR2 = 0;
	HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, RESET);

	TIM12->CCR1 = 0;
//	TIM12->CCR2 = 0;
	HAL_GPIO_WritePin(TOGGLING_MTR_EN_GPIO_Port, TOGGLING_MTR_EN_Pin, RESET);

	TIM1->CCR3 = 0;
//	TIM1->CCR4 = 0;
	HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, RESET);
}

void Press_DC_Set_PWM(float pwm_val, Motor_Direction Direction)
{
#if		CYTRON_DRV == 1
	if(Direction == CLOCKWISE)
	{
#if		!BOARD_V2_0
		HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, SET);
#elif 	BOARD_V2_0
		HAL_GPIO_WritePin(PRESS_MTR_DIR_GPIO_Port, PRESS_MTR_DIR_Pin, RESET);
#endif
	}
	if(Direction == ANTICLOCKWISE)
	{
#if		!BOARD_V2_0
		HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, RESET);
#elif	BOARD_V2_0
		HAL_GPIO_WritePin(PRESS_MTR_DIR_GPIO_Port, PRESS_MTR_DIR_Pin, SET);
#endif
	}
	TIM1->CCR3 = pwm_val;
#elif		CYTRON_DRV == 0
	HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		TIM1->CCR3 = pwm_val;
		TIM1->CCR4 = 0;
	}
	else if(Direction == ANTICLOCKWISE)
	{
		TIM1->CCR3 = 0;
		TIM1->CCR4 = pwm_val;		//Modified on 02-11-20 with PCBA
	}
#elif		CYTRON_DRV == 2
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(SPARE_DC_MTR_ENA_GPIO_Port, SPARE_DC_MTR_ENA_Pin, SET);
		HAL_GPIO_WritePin(SPARE_DC_MTR_ENB_GPIO_Port, SPARE_DC_MTR_ENB_Pin, RESET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(SPARE_DC_MTR_ENA_GPIO_Port, SPARE_DC_MTR_ENA_Pin, RESET);
		HAL_GPIO_WritePin(SPARE_DC_MTR_ENB_GPIO_Port, SPARE_DC_MTR_ENB_Pin, SET);
	}
	TIM3->CCR3 = pwm_val;
#endif
}

void Blade_DC_Set_PWM(float pwm_val, Motor_Direction Direction)
{
	if(Direction == CLOCKWISE)
	{
#if		!BOARD_V2_0
		HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, SET);
#elif	BOARD_V2_0
		HAL_GPIO_WritePin(KNEAD_DIR_GPIO_Port, KNEAD_DIR_Pin, SET);
#endif
		TIM4->CCR1 = pwm_val;
//		TIM4->CCR2 = 0;
	}
	else if(Direction == ANTICLOCKWISE)
	{
#if		!BOARD_V2_0
		HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, RESET);
#elif	BOARD_V2_0
		HAL_GPIO_WritePin(KNEAD_DIR_GPIO_Port, KNEAD_DIR_Pin, RESET);
#endif
//		HAL_GPIO_WritePin(KNEAD_DIR_GPIO_Port, KNEAD_DIR_Pin, RESET);
		TIM4->CCR1 = pwm_val;
//		TIM4->CCR2 = pwm_val;
	}
		/*HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, SET);
		if(Direction == CLOCKWISE)
		{
			TIM4->CCR1 = pwm_val;
			TIM4->CCR2 = 0;
		}
		else if(Direction == ANTICLOCKWISE)
		{
			TIM4->CCR1 = 0;
			TIM4->CCR2 = pwm_val;
		}*/
}

void Toggling_DC_Set_PWM(float pwm_val, Motor_Direction Direction)
{
	/*HAL_GPIO_WritePin(TOGGLING_MTR_EN_GPIO_Port, TOGGLING_MTR_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		TIM12->CCR1 = 0;
		TIM12->CCR2 = pwm_val;
	}
	else if(Direction == ANTICLOCKWISE)
	{
		TIM12->CCR1 = pwm_val;
		TIM12->CCR2 = 0;
	}*/
#if 1
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(TOGGLING_MTR_DIR_GPIO_Port, TOGGLING_MTR_DIR_Pin, RESET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(TOGGLING_MTR_DIR_GPIO_Port, TOGGLING_MTR_DIR_Pin, SET);
	}
	TIM12->CCR1 = pwm_val;
#endif
}

void Press_DC_Motor_Stop()
{
#if	CYTRON_DRV == 1
	TIM1->CCR3 = 0;
//	TIM1->CCR4 = 100;
//	HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, RESET);
#elif	CYTRON_DRV == 0
	TIM1->CCR3 = 100;
	TIM1->CCR4 = 100;
	HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, RESET);
#elif CYTRON_DRV == 2
	HAL_GPIO_WritePin(SPARE_DC_MTR_ENA_GPIO_Port, SPARE_DC_MTR_ENA_Pin, RESET);
	HAL_GPIO_WritePin(SPARE_DC_MTR_ENB_GPIO_Port, SPARE_DC_MTR_ENB_Pin, RESET);
	TIM3->CCR3 = 0;
#endif
}

void Blade_DC_Motor_Stop()
{
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, RESET);
//	HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, RESET);
}
void Toggling_DC_Motor_Stop()
{
	TIM12->CCR1 = 0;
//    TIM12->CCR2 = 100;
//	HAL_GPIO_WritePin(TOGGLING_MTR_EN_GPIO_Port, TOGGLING_MTR_EN_Pin, SET);
}


void PWM_Channels_Stop(void)
{
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);		//Toggling Motor RPWM
//	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);		//Toggling Motor LPWM
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);		//Kneading Blade Motor RPWM
//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);		//Kneading Blade MOtor  LPWM
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);		//Press Motor RPWM
//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);		//Press Motor LPWM
	HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);		//Agitator Motor PWM
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);		//Spare Motor
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);		//Spare Motor
//	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);		//Spare Motor PWM
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
//	HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, RESET);

	TIM12->CCR1 = 0;
//	TIM12->CCR2 = 0;
//	HAL_GPIO_WritePin(TOGGLING_MTR_EN_GPIO_Port, TOGGLING_MTR_EN_Pin, RESET);

	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
//	HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, RESET);
}
//void DC_Motor_Stop(DC_Motor Motor_no)
//{
//	switch(Motor_no)
//	{
//		case PRESS:
//			HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, RESET);
//			TIM1->CCR3 = 0;
//			TIM1->CCR4 = 0;
//			break;
//		case BLADE:
//			HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, RESET);
//			TIM4->CCR1 = 0;
//			TIM4->CCR2 = 0;
//			break;
//		case TOGGLING:
//			HAL_GPIO_WritePin(TOGGLING_MTR_EN_GPIO_Port, TOGGLING_MTR_EN_Pin, RESET);
//			TIM12->CCR1 = 0;
//			TIM12->CCR2 = 0;
//			break;
//	}
	/*if(Motor_no == PRESS)
	{
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
	}
	else if(Motor_no == TOGGLING)
	{
		TIM13->CCR1 = 0;
		TIM14->CCR1 = 0;
	}
	else if(Motor_no == BLADE)
	{
		TIM5->CCR3 = 0;
		TIM5->CCR4 = 0;
	}*/
//}

/* @brief: This function is to run the DC motor of DRB driver
 * @params:  pwm->PWM duty cycle at which the motor must run
 * 			 direction->The direction of the motor
 * @return: NOTHING
 * */
void Agitator_MTR_Run(float pwm, Motor_Direction direction)
{
	switch(direction)
	{
		case CLOCKWISE:
			HAL_GPIO_WritePin(AGITATOR_MTR_DIR_GPIO_Port, AGITATOR_MTR_DIR_Pin, RESET);
			TIM13->CCR1 = pwm;
			break;
		case ANTICLOCKWISE:
			HAL_GPIO_WritePin(AGITATOR_MTR_DIR_GPIO_Port, AGITATOR_MTR_DIR_Pin, SET);
			TIM13->CCR1 = 0;
			break;
		default:
			break;
	}
}
/* @brief: To stop the Agitator Motor
 * @return: NOTHING*/
void Agitator_MTR_Stop(void)
{
	TIM13->CCR1 = 0;
}

