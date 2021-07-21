/*
 * step_motor.c
 *
 *  Created on: Sep 21, 2020
 *      Author: Admin
 */
#include"step_motor.h"

/**/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

/*Variable Declaration*/
#if 0
uint32_t step_val = 0;
volatile float temp1,temp2;
volatile uint32_t rpm,prescaler,frequency;
uint16_t RPM_COUNT = 1;
float step_angle;
uint8_t TIM3_flag=0,TIM7_flag=0;
uint8_t leadscrew_step = 0, flour_step = 0, water_step = 0, oil_step = 0, ejector_step =0;
extern volatile uint8_t dispense_complete;
volatile uint8_t flour_disp =0, water_disp = 0, oil_disp = 0;
volatile uint8_t time_1ms_completed;
//extern Dispensing_Stepper;
//extern Dispensing_Stepper disp_motor_interrupt;
/*Variable Declaration*/
/**/
/* @brief: This function is to run the Stepper motors
 * @params:  RPM->Motor RPM
 * 			 Step->Number of steps to rotate
 * 			 Direction->Direction of the stepper motor
 * 			 MotorSel->Stepper motor to rotate
 * 			 *Oil Stepper pump
 * 			 *Flour dispensing stepper motor
 * 			 *Ejector Stepper motor
 * @return: NOTHING
 * */
void Stepper_Disp_Rotate(unsigned int RPM, unsigned int Step, Motor_Direction Direction, Dispensing_Stepper MotorSel)
{
	switch(MotorSel)
	{
		case FLOUR_DISP   :
			HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin, SET);
			if(Direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(FLOUR_DISP_DIR_GPIO_Port, FLOUR_DISP_DIR_Pin, SET);
			}
			else if(Direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(FLOUR_DISP_DIR_GPIO_Port, FLOUR_DISP_DIR_Pin, RESET);
			}
//			flour_step = 1;
			flour_disp = 1;
			disp_motor_interrupt = FLOUR_DISP;
			break;
		case WATER_DISP :
			HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port, WATER_DISP_EN_Pin, SET);
			if(Direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(WATER_DISP_DIR_GPIO_Port, WATER_DISP_DIR_Pin, SET);
			}
			else if(Direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(WATER_DISP_DIR_GPIO_Port, WATER_DISP_DIR_Pin, RESET);
			}
			water_disp = 1;
//			water_step = 1;
			disp_motor_interrupt = WATER_DISP;
			break;
		case OIL_DISP :
			HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin, SET);
			if(Direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(OIL_DISP_DIR_GPIO_Port, OIL_DISP_DIR_Pin, SET);
			}
			else if(Direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(OIL_DISP_DIR_GPIO_Port, OIL_DISP_DIR_Pin, RESET);
			}
			oil_disp = 1;
//			oil_step = 1;
			disp_motor_interrupt = OIL_DISP;
			break;
		default :
			break;
	}
	dispense_complete = 1;
	step_angle=HALF_STEP;

	step_val = Step*2;
	TIM7_flag=0;


	HAL_TIM_Base_Start_IT(&htim14);
	if(RPM>300)
	{
		for(RPM_COUNT=1;RPM_COUNT<RPM;RPM_COUNT++)
		{
			temp1=(RPM_COUNT*360)/(step_angle*60);
			temp1=temp1*2;
			frequency=(int)temp1;
			prescaler=(int)Timer_clock/(period*frequency);
			TIM14->CCR1=20;
			TIM14->ARR=(period-1);
			TIM14->PSC=(prescaler-1);
			HAL_Delay(1);
		}
	}
	temp1=(RPM*360)/(step_angle*60);
	temp1=temp1*2;
	frequency=(int)temp1;
	prescaler=(int)Timer_clock/(period*frequency);
	TIM14->CCR1=20;
	TIM14->ARR=(period-1);
	TIM14->PSC=(prescaler-1);
}
/* @brief: This function is to run the Stepper motors
 * @params:  RPM->Motor RPM
 * 			 Step->Number of steps to rotate
 * 			 Direction->Direction of the stepper motor
 * 			 MotorSel->Stepper motor to rotate
 * 			 *Kneader Leadscrew
 * @return: NOTHING
 * */
void Stepper_Kneader_Rotate(unsigned int RPM, unsigned int Step, Motor_Direction Direction)
{
	leadScrewMotor = LEAD_ROTATE;
	HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port, LEADSCREW_DIR_Pin, SET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port, LEADSCREW_DIR_Pin, RESET);
	}
	leadscrew_step = 1;

	step_angle=HALF_STEP;

	step_val = Step*2;
	TIM7_flag=0;


	HAL_TIM_Base_Start_IT(&htim15);
	if(RPM>300)
	{
		if(time_1ms_completed == 1)
		{
//			if(RPM_COUNT < RPM)
			for(RPM_COUNT = 1; RPM_COUNT < RPM; RPM_COUNT++)
			{
				temp1=(RPM_COUNT*360)/(step_angle*60);
				temp1=temp1*2;
				frequency=(int)temp1;
				prescaler=(int)Timer_clock/(period*frequency);
				TIM15->CCR1=20;
				TIM15->ARR=(period-1);
				TIM15->PSC=(prescaler-1);
//				RPM_COUNT += 1;
	//			HAL_Delay(1);
			}
			time_1ms_completed = 0;
		}

//		for(RPM_COUNT=1;RPM_COUNT<RPM;RPM_COUNT++)
//		{
//			temp1=(RPM_COUNT*360)/(step_angle*60);
//			temp1=temp1*2;
//			frequency=(int)temp1;
//			prescaler=(int)Timer_clock/(period*frequency);
//			TIM15->CCR1=20;
//			TIM15->ARR=(period-1);
//			TIM15->PSC=(prescaler-1);
//			HAL_Delay(1);
//		}
	}
	temp1=(RPM*360)/(step_angle*60);
	temp1=temp1*2;
	frequency=(int)temp1;
	prescaler=(int)Timer_clock/(period*frequency);
	TIM15->CCR1=20;
	TIM15->ARR=(period-1);
	TIM15->PSC=(prescaler-1);
}
void Stepper_Leadscrew_RotateTop(unsigned int RPM, Motor_Direction Direction)
{
	HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port, LEADSCREW_DIR_Pin, SET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port, LEADSCREW_DIR_Pin, RESET);
	}
	leadscrew_step = 1;

	step_angle=HALF_STEP;

//	step_val = Step*2;


	HAL_TIM_Base_Start_IT(&htim15);
	if(RPM>300)
	{
		if(time_1ms_completed == 1)
		{
//			if(RPM_COUNT < RPM)
			for(RPM_COUNT = 1; RPM_COUNT < RPM; RPM_COUNT++)
			{
				temp1=(RPM_COUNT*360)/(step_angle*60);
				temp1=temp1*2;
				frequency=(int)temp1;
				prescaler=(int)Timer_clock/(period*frequency);
				TIM15->CCR1=20;
				TIM15->ARR=(period-1);
				TIM15->PSC=(prescaler-1);
//				RPM_COUNT += 1;
	//			HAL_Delay(1);
			}
			time_1ms_completed = 0;
		}

//		for(RPM_COUNT=1;RPM_COUNT<RPM;RPM_COUNT++)
//		{
//			temp1=(RPM_COUNT*360)/(step_angle*60);
//			temp1=temp1*2;
//			frequency=(int)temp1;
//			prescaler=(int)Timer_clock/(period*frequency);
//			TIM15->CCR1=20;
//			TIM15->ARR=(period-1);
//			TIM15->PSC=(prescaler-1);
//			HAL_Delay(1);
//		}
	}
	temp1=(RPM*360)/(step_angle*60);
	temp1=temp1*2;
	frequency=(int)temp1;
	prescaler=(int)Timer_clock/(period*frequency);
	TIM15->CCR1=20;
	TIM15->ARR=(period-1);
	TIM15->PSC=(prescaler-1);
	leadScrewMotor = LEAD_TOP;
}

void Stepper_Leadscrew_RotateBottom(unsigned int RPM, Motor_Direction Direction)
{
	HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port, LEADSCREW_DIR_Pin, SET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port, LEADSCREW_DIR_Pin, RESET);
	}
	leadscrew_step = 1;

	step_angle=HALF_STEP;

//	step_val = Step*2;
//	TIM7_flag=0;


	HAL_TIM_Base_Start_IT(&htim15);
	if(RPM>300)
	{
		if(time_1ms_completed == 1)
		{
//			if(RPM_COUNT < RPM)
			for(RPM_COUNT = 1; RPM_COUNT < RPM; RPM_COUNT++)
			{
				temp1=(RPM_COUNT*360)/(step_angle*60);
				temp1=temp1*2;
				frequency=(int)temp1;
				prescaler=(int)Timer_clock/(period*frequency);
				TIM15->CCR1=20;
				TIM15->ARR=(period-1);
				TIM15->PSC=(prescaler-1);
//				RPM_COUNT += 1;
	//			HAL_Delay(1);
			}
			time_1ms_completed = 0;
		}

//		for(RPM_COUNT=1;RPM_COUNT<RPM;RPM_COUNT++)
//		{
//			temp1=(RPM_COUNT*360)/(step_angle*60);
//			temp1=temp1*2;
//			frequency=(int)temp1;
//			prescaler=(int)Timer_clock/(period*frequency);
//			TIM15->CCR1=20;
//			TIM15->ARR=(period-1);
//			TIM15->PSC=(prescaler-1);
//			HAL_Delay(1);
//		}
	}
	temp1=(RPM*360)/(step_angle*60);
	temp1=temp1*2;
	frequency=(int)temp1;
	prescaler=(int)Timer_clock/(period*frequency);
	TIM15->CCR1=20;
	TIM15->ARR=(period-1);
	TIM15->PSC=(prescaler-1);
	leadScrewMotor = LEAD_BOTTOM;
}
/* @brief: This function is to run the Ejector Stepper motor
 * @params:  RPM->Motor RPM
 * 			 Step->Number of steps to rotate
 * 			 Direction->Direction of the stepper motor
 * 			 MotorSel->Stepper motor to rotate
 * 			 *Ejector Stepper Motor
 * @return: NOTHING
 * */
void Stepper_Ejector_Rotate(unsigned int RPM, unsigned int Step, Motor_Direction Direction)
{
	ejectorMotor = EJECTOR_ROTATE;
	HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin, SET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin, RESET);
	}
//	leadscrew_step = 1;
	dispense_complete = 1;

	step_angle=HALF_STEP;

	step_val = Step*2;
	TIM7_flag=0;


	HAL_TIM_Base_Start_IT(&htim17);
	if(RPM>300)
	{
		for(RPM_COUNT=1;RPM_COUNT<RPM;RPM_COUNT++)
		{
			temp1=(RPM_COUNT*360)/(step_angle*60);
			temp1=temp1*2;
			frequency=(int)temp1;
			prescaler=(int)Timer_clock/(period*frequency);
			TIM17->CCR1=20;
			TIM17->ARR=(period-1);
			TIM17->PSC=(prescaler-1);
			HAL_Delay(1);
		}
	}
	temp1=(RPM*360)/(step_angle*60);
	temp1=temp1*2;
	frequency=(int)temp1;
	prescaler=(int)Timer_clock/(period*frequency);
	TIM17->CCR1=20;
	TIM17->ARR=(period-1);
	TIM17->PSC=(prescaler-1);
}
void Stepper_Ejector_Homing(unsigned int RPM, Motor_Direction Direction)
{
	ejectorMotor = EJECTOR_HOME;
	HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin, SET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin, RESET);
	}
//	leadscrew_step = 1;
	dispense_complete = 1;

	step_angle=HALF_STEP;

//	step_val = Step*2;
//	TIM7_flag=0;


	HAL_TIM_Base_Start_IT(&htim17);
	if(RPM>300)
	{
		for(RPM_COUNT=1;RPM_COUNT<RPM;RPM_COUNT++)
		{
			temp1=(RPM_COUNT*360)/(step_angle*60);
			temp1=temp1*2;
			frequency=(int)temp1;
			prescaler=(int)Timer_clock/(period*frequency);
			TIM17->CCR1=20;
			TIM17->ARR=(period-1);
			TIM17->PSC=(prescaler-1);
			HAL_Delay(1);
		}
	}
	temp1=(RPM*360)/(step_angle*60);
	temp1=temp1*2;
	frequency=(int)temp1;
	prescaler=(int)Timer_clock/(period*frequency);
	TIM17->CCR1=20;
	TIM17->ARR=(period-1);
	TIM17->PSC=(prescaler-1);
}
void Stepper_Ejector_End(unsigned int RPM, Motor_Direction Direction)
{
	ejectorMotor = EJECTOR_END;
	HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, SET);
	if(Direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin, SET);
	}
	else if(Direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin, RESET);
	}
//	leadscrew_step = 1;
	dispense_complete = 1;

	step_angle=HALF_STEP;

//	step_val = Step*2;
//	TIM7_flag=0;


	HAL_TIM_Base_Start_IT(&htim17);
	if(RPM>300)
	{
		for(RPM_COUNT=1;RPM_COUNT<RPM;RPM_COUNT++)
		{
			temp1=(RPM_COUNT*360)/(step_angle*60);
			temp1=temp1*2;
			frequency=(int)temp1;
			prescaler=(int)Timer_clock/(period*frequency);
			TIM17->CCR1=20;
			TIM17->ARR=(period-1);
			TIM17->PSC=(prescaler-1);
			HAL_Delay(1);
		}
	}
	temp1=(RPM*360)/(step_angle*60);
	temp1=temp1*2;
	frequency=(int)temp1;
	prescaler=(int)Timer_clock/(period*frequency);
	TIM17->CCR1=20;
	TIM17->ARR=(period-1);
	TIM17->PSC=(prescaler-1);
}
/* @brief: This function is to stop the Dispensing Stepper motors
 * @params: NOTHING
 * @return: NOTHING
 * */
void StopDispensingMotors(Dispensing_Stepper Motor)
{
	switch(disp_motor_interrupt)
	{
		case FLOUR_DISP:
			HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin, RESET);
			HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin, RESET);   // Pulse pin since EN is always high
		  break;
		case WATER_DISP:
			HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port, WATER_DISP_DIR_Pin, RESET);
			HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin, RESET);   // Pulse pin since EN is always high
		  break;
		case OIL_DISP:
			HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin, RESET);
			HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin, RESET);   // Pulse pin since EN is always high
		  break;
		default:
			break;
	}

}
/* @brief: This function is to stop the Leadscrew Stepper motors
 * @params: NOTHING
 * @return: NOTHING
 * */
void StopLeadscrewMotor(void)
{
	HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, RESET);
	HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin, RESET);   // Pulse pin since EN is always high
}
/* @brief: This function is to stop the Ejector Stepper motors
 * @params: NOTHING
 * @return: NOTHING
 * */
void StopEjectorMotor(void)
{
	HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, RESET);
	HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin, RESET);   // Pulse pin since EN is always high
}

#endif
