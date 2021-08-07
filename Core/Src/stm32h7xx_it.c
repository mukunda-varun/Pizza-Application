/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "step_motor.h"
#include "hx711.h"
#include "MAX_31855.h"
#include "Android_Interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//uint32_t value = 0;
//uint8_t i=0;
extern uint32_t time_measure;
extern uint32_t time_measure_debug_cnt;
extern uint8_t ref_msg;
extern uint16_t Temperature_value_Sensor_top_pan,Temperature_value_Sensor_bottom_pan;
extern uint8_t bakingValue_flag;
extern uint32_t timeleft_count;
extern uint32_t baking_timer6_counter;
extern uint32_t led_toggle ;
extern volatile uint32_t usb_cnt ;
extern volatile uint8_t id ;

extern HX711 *OIL_LOADCELL;
extern HX711  *FLOUR_LOADCELL;
extern HX711  *WATER_LOADCELL;
extern volatile uint32_t PosDistance;
extern Stepper_Motor Ejecter_Stepper_Motor,
                     Knead_screw_Stepper_Motor,
                     Flour_Stepper_Motor,
                     Oil_Stepper_Motor,
                     Water_Stepper_Motor;
extern uint16_t sum;
extern uint32_t water_pulse_count;
extern volatile uint32_t water_value;
extern Pressing_Baking_t Pressing_Baking;
extern uint8_t complete_process_done;
extern uint8_t kneading_state_android;
extern volatile uint8_t press_process_percent;
extern volatile uint32_t bakingValue_left_counter;
extern countVal  timerCount;
extern proxCnt proximityCnt;
extern proximity pnpProximityValues;
extern errorState_t processErrorState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

extern void Update_end_of_process_status(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	water_pulse_count++;
	if((water_pulse_count >= (water_value) && (machine_initialization_state == machine_initialization_complete))  || Water_Stepper_Motor.steps_counter >= Water_Stepper_Motor.total_no_of_steps + WATER_DISPENSE_STEPS_TOLERANCE)
	{
		if((Cleaning_state_t != water_pump_priming_run_init && Cleaning_state_t != water_pump_priming_run_wait) && ((Cleaning_state_t != water_pump_cleaning_run_init && Cleaning_state_t != water_pump_cleaning_run_wait)))
		{
			HAL_TIM_Base_Stop_IT(&htim7);
			dispenseStateForInterrupt=Idle_Dispense;
		}
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

//	Temperature_control(Pressing_Baking.Top_Pan_Cut_off_Temp,Pressing_Baking.Bottom_Pan_Cut_off_Temp);

//	Temperature_value_Sensor_top_pan = 30;
//	Temperature_value_Sensor_bottom_pan = 30;
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
	Knead_screw_Stepper_Motor.steps_counter++;
	HAL_GPIO_TogglePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin);
	//knead_screw_homing_or_top_end_limit_state,knead_screw_rear_end_or_bottom_end_limit_state,Knead_movement_to_xx_mm_state;
	if((knead_screw_rear_end_or_bottom_end_limit_state!=knead_screw_idle)&&(Knead_movement_to_xx_mm_state==knead_screw_idle))
	{
		if((Knead_screw_Stepper_Motor.direction == CLOCKWISE)&&(pnpProximityValues.leadscrewBottomPositionLimit == DETECTED))
		 {
			HAL_TIM_Base_Stop_IT(&htim14);
		 }
	}
	if((Knead_movement_to_xx_mm_state!=knead_screw_idle)&&(knead_screw_homing_or_top_end_limit_state==knead_screw_idle))
	{
		if((Knead_screw_Stepper_Motor.direction == CLOCKWISE)&&((Knead_screw_Stepper_Motor.steps_counter>=Knead_screw_Stepper_Motor.total_no_of_steps))&& (pnpProximityValues.leadscrewBottomPositionLimit == DETECTED))//||(HAL_GPIO_ReadPin(EJECTOR_END_POS_GPIO_Port, EJECTOR_END_POS_Pin)!=(uint8_t)SET)||(HAL_GPIO_ReadPin(KNEAD_BOTTOM_POS_GPIO_Port, KNEAD_BOTTOM_POS_Pin)!=(uint8_t)SET)))
		 {
				 HAL_TIM_Base_Stop_IT(&htim14);
		 }
		 if((Knead_screw_Stepper_Motor.direction == ANTICLOCKWISE)&&((Knead_screw_Stepper_Motor.steps_counter>=Knead_screw_Stepper_Motor.total_no_of_steps))&&(pnpProximityValues.leadscrewTopPositionLimit == DETECTED))
		 {
			 HAL_TIM_Base_Stop_IT(&htim14);
		 }
	}
	if((knead_screw_homing_or_top_end_limit_state!=knead_screw_idle)&&(Knead_movement_to_xx_mm_state==knead_screw_idle))
	{
		 if((Knead_screw_Stepper_Motor.direction == ANTICLOCKWISE)&&(pnpProximityValues.leadscrewTopPositionLimit == DETECTED))
		 {
			 HAL_TIM_Base_Stop_IT(&htim14);
		 }
	}
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	timerCount.togglingTimerCnt++;
	timerCount.kneadingTimerCnt++;
	timerCount.ejectorTimerCnt++;
	timerCount.pressTimerCnt++;

	if((bakingValue_flag == 1) && (Main_process_state == Main_press || Main_process_state == Main_Baking_Wait))
	{
		timerCount.bakingTimerCnt++;
		timeleft_count++;
		if(timeleft_count>=1000)
		{
			bakingValue_left_counter--;
			timeleft_count=0;
		}
	}
	usb_cnt++;
	if(usb_cnt >= 1000 && complete_process_done == 0)
	{
		usb_cnt = 0;
		id++;
		if(id>=4)
		id=0;
		Update_Status(id);
	}
	if(complete_process_done == 1)
	{
		complete_process_done = 2;
		Update_end_of_process_status();
	}
	led_toggle++;
	if(led_toggle  >= 1000)
	{
		led_toggle = 0;
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_TogglePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}

	if(Cleaning_state_t == water_pump_cleaning_run_wait || Cleaning_state_t == water_pump_cleaning_run_wait
			|| Cleaning_state_t == water_pump_priming_run_init || Cleaning_state_t == water_pump_priming_run_wait)
	{
		timerCount.waterMaintenance++;
	}
	if(Cleaning_state_t == oil_pump_cleaning_run_wait || Cleaning_state_t == oil_pump_cleaning_run_wait
				|| Cleaning_state_t == oil_pump_priming_run_init || Cleaning_state_t == oil_pump_priming_run_wait)
	{
		timerCount.oilMaintenance++;
	}
	if(Cleaning_state_t == flour_motor_cleaning_run_wait || Cleaning_state_t == flour_motor_cleaning_run_wait
				|| Cleaning_state_t == flour_motor_priming_run_init || Cleaning_state_t == flour_motor_priming_run_wait)
	{
		timerCount.flourMaintenance++;
	}


	if(time_measure)
	{
		if(time_measure_debug_cnt++  >= 5000)
		{
			time_measure_debug_cnt = 0;
			time_measure = 0;
			led_toggle = 0;
//			Send_ADR_Response(ref_msg,2);
		}
	}
	/******************************Proximity******************************/
	/*Press Top Proximity*/
	if(HAL_GPIO_ReadPin(PRESS_TOP_POS_GPIO_Port, PRESS_TOP_POS_Pin)!=(uint8_t)SET)
	{
		proximityCnt.pressProxCnt++;
		if(proximityCnt.pressProxCnt >= PROXIMITY_DEBOUNCE_CNT)
		{
			pnpProximityValues.pressTopPositionLimit = DETECTED;
		}
	}
	else
	{
		proximityCnt.pressProxCnt = 0;
		pnpProximityValues.pressTopPositionLimit = NOT_DETECTED;
	}
	/*Press Top Proximity*/
	/*Kneader Top Proximity*/
#if	MACHINE_1
	if(HAL_GPIO_ReadPin(KNEAD_TOP_POS_GPIO_Port, KNEAD_TOP_POS_Pin)!=(uint8_t)SET)
#elif !MACHINE_1
	if(HAL_GPIO_ReadPin(KNEAD_TOP_POS_GPIO_Port, KNEAD_TOP_POS_Pin)!=(uint8_t)SET)
#endif
	{
		proximityCnt.leadTopProxCnt++;
		if(proximityCnt.leadTopProxCnt >= PROXIMITY_DEBOUNCE_CNT)
		{
			pnpProximityValues.leadscrewTopPositionLimit = DETECTED;
		}
	}
	else
	{
		proximityCnt.leadTopProxCnt = 0;
		pnpProximityValues.leadscrewTopPositionLimit = NOT_DETECTED;
	}
	/*Kneader Top Proximity*/
	/*Kneader Bottom Proximity*/
#if	MACHINE_1
	if(HAL_GPIO_ReadPin(KNEAD_BOTTOM_POS_GPIO_Port, KNEAD_BOTTOM_POS_Pin)!=(uint8_t)SET)
#elif !MACHINE_1
//	if(HAL_GPIO_ReadPin(SPARE_POS2_GPIO_Port, SPARE_POS2_Pin)!=(uint8_t)SET)
	if(HAL_GPIO_ReadPin(KNEAD_BOTTOM_POS_GPIO_Port, KNEAD_BOTTOM_POS_Pin)!=(uint8_t)SET)
#endif
	{
		proximityCnt.leadBtmProxCnt++;
		if(proximityCnt.leadBtmProxCnt >= PROXIMITY_DEBOUNCE_CNT)
		{
			pnpProximityValues.leadscrewBottomPositionLimit = DETECTED;
		}
	}
	else
	{
		proximityCnt.leadBtmProxCnt = 0;
		pnpProximityValues.leadscrewBottomPositionLimit = NOT_DETECTED;
	}
	/*Kneader Bottom Proximity*/
	/*Ejector Start Proximity*/
	if(HAL_GPIO_ReadPin(EJECTOR_START_POS_GPIO_Port, EJECTOR_START_POS_Pin)!=(uint8_t)SET)
	{
		proximityCnt.ejectorStartProxCnt++;
		if(proximityCnt.ejectorStartProxCnt >= PROXIMITY_DEBOUNCE_CNT)
		{
			pnpProximityValues.ejectorStartPositionLimit = DETECTED;
		}
	}
	else
	{
		proximityCnt.ejectorStartProxCnt = 0;
		pnpProximityValues.ejectorStartPositionLimit = NOT_DETECTED;
	}
	/*Ejector Start Proximity*/
	/*Ejector End Proximity*/
	if(HAL_GPIO_ReadPin(EJECTOR_END_POS_GPIO_Port, EJECTOR_END_POS_Pin)!=(uint8_t)SET)
	{
		proximityCnt.ejectorEndProxCnt++;
		if(proximityCnt.ejectorEndProxCnt >= PROXIMITY_DEBOUNCE_CNT)
		{
			pnpProximityValues.ejectorFrontEndPositionLimit = DETECTED;
		}
	}
	else
	{
		proximityCnt.ejectorEndProxCnt = 0;
		pnpProximityValues.ejectorFrontEndPositionLimit = NOT_DETECTED;
	}
	/*Ejector End Proximity*/

#if INTERLOCK_EN ==  1
	/*BOTTOM DOOR Proximity*/
	if(HAL_GPIO_ReadPin(SPARE_POS1_GPIO_Port, SPARE_POS1_Pin)!=(uint8_t)SET)
	{
		proximityCnt.bottomDoorProxCnt++;
		if(proximityCnt.bottomDoorProxCnt >= PROXIMITY_DEBOUNCE_CNT)
		{
			if(processErrorState.bottomDoorError == 1)
			{
				if(ejecter_home_position_state != ejecter_idle && ejecter_home_position_state != ejecter_completed)
				{
					HAL_TIM_Base_Stop_IT(&htim15);
					Ejecter_Stepper_Motor.rpm_counter=0;
					Ejecter_Stepper_Motor.steps_counter=0;
					Ejecter_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
					HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
					ejecter_home_position_state = ejecter_start;
				}
				else if(ejecter_front_end_limit_position_state != ejecter_idle && ejecter_front_end_limit_position_state != ejecter_completed)
				{
					HAL_TIM_Base_Stop_IT(&htim15);
					Ejecter_Stepper_Motor.rpm_counter=0;
					Ejecter_Stepper_Motor.steps_counter=0;
					Ejecter_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
					HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
					ejecter_front_end_limit_position_state = ejecter_start;
				}
				processErrorState.bottomDoorError = 0;
			}
			pnpProximityValues.bottomDoorProxLimit = DETECTED;
		}
	}
	else
	{
		processErrorState.bottomDoorError = 1;
		proximityCnt.bottomDoorProxCnt = 0;
		pnpProximityValues.bottomDoorProxLimit = NOT_DETECTED;
	}
	/*BOTTOM DOOR Proximity*/
	/*TOP DOOR Proximity*/
	if(HAL_GPIO_ReadPin(SPARE_POS2_GPIO_Port, SPARE_POS2_Pin)!=(uint8_t)SET)
	{
		proximityCnt.topDoorProxCnt++;
		if(proximityCnt.topDoorProxCnt >= PROXIMITY_DEBOUNCE_CNT)
		{
			processErrorState.topDoorError = 0;
			pnpProximityValues.topDoorProxLimit = DETECTED;
		}
	}
	else
	{
		processErrorState.topDoorError = 1;
		proximityCnt.topDoorProxCnt = 0;
		pnpProximityValues.topDoorProxLimit = NOT_DETECTED;
	}
	/*TOP DOOR Proximity*/
#endif

	/******************************Proximity******************************/

	/* USER CODE END PV */
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac1);
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	switch(dispenseStateForInterrupt)
	{

		case Idle_Dispense:

			break;
		case flourDispense:
			kneading_state_android = 2;
			Flour_Stepper_Motor.steps_counter++;
			HAL_GPIO_TogglePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin);
			if(Flour_Stepper_Motor.steps_counter >= Flour_Stepper_Motor.total_no_of_steps)
			{
				if((Cleaning_state_t != flour_motor_priming_run_init && Cleaning_state_t != flour_motor_priming_run_wait))
				{
					HAL_TIM_Base_Stop_IT(&htim7);
					dispenseStateForInterrupt=Idle_Dispense;
				}
			}
			break;
		case waterDispense:
			kneading_state_android = 3;
			Water_Stepper_Motor.steps_counter++;
			HAL_GPIO_TogglePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin);
//				if(Water_Stepper_Motor.steps_counter >= Water_Stepper_Motor.total_no_of_steps)
//				{
//					HAL_TIM_Base_Stop_IT(&htim7);
//					dispenseStateForInterrupt=Idle_Dispense;
//				}
			break;
		case oilDispense:
			kneading_state_android = 1;
			Oil_Stepper_Motor.steps_counter++;
			HAL_GPIO_TogglePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin);
			if(Oil_Stepper_Motor.steps_counter >= Oil_Stepper_Motor.total_no_of_steps)
			{
				if(Cleaning_state_t != oil_pump_priming_run_init && Cleaning_state_t != oil_pump_priming_run_wait && Cleaning_state_t != oil_pump_cleaning_run_init && Cleaning_state_t != oil_pump_cleaning_run_wait)
				{
					HAL_TIM_Base_Stop_IT(&htim7);
					dispenseStateForInterrupt=Idle_Dispense;
				}
			}
			break;
	}

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles TIM15 global interrupt.
  */
void TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_IRQn 0 */

  /* USER CODE END TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM15_IRQn 1 */

	Ejecter_Stepper_Motor.steps_counter++;
#if	!MACHINE_1
	HAL_GPIO_TogglePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin);
#elif	MACHINE_1
	HAL_GPIO_TogglePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin);
#endif

	if((ejecter_front_end_limit_position_state!=ejecter_idle)&&(ejecter_movment_to_xx_mm_state==ejecter_idle))
	{
	 if((Ejecter_Stepper_Motor.direction == CLOCKWISE)&&(pnpProximityValues.ejectorFrontEndPositionLimit == DETECTED))
		 {
			HAL_TIM_Base_Stop_IT(&htim15);
		 }
	}
	if((ejecter_movment_to_xx_mm_state!=ejecter_idle)&&(ejecter_front_end_limit_position_state==ejecter_idle))
	{
		if((Ejecter_Stepper_Motor.direction ==  CLOCKWISE)&&((Ejecter_Stepper_Motor.steps_counter>=Ejecter_Stepper_Motor.total_no_of_steps)&&(pnpProximityValues.ejectorFrontEndPositionLimit == DETECTED)))
		 {
				 HAL_TIM_Base_Stop_IT(&htim15);
		 }
		 if((Ejecter_Stepper_Motor.direction == ANTICLOCKWISE)&&(Ejecter_Stepper_Motor.steps_counter>=Ejecter_Stepper_Motor.total_no_of_steps)&&(pnpProximityValues.ejectorStartPositionLimit == DETECTED))
		 {
			 HAL_TIM_Base_Stop_IT(&htim15);
		 }
	}
	if((ejecter_home_position_state!=ejecter_idle)&&(ejecter_movment_to_xx_mm_state==ejecter_idle))
	{
		 if((Ejecter_Stepper_Motor.direction == /*CLOCKWISE*/ANTICLOCKWISE)&&
			(pnpProximityValues.ejectorStartPositionLimit == DETECTED))
			 {
				 HAL_TIM_Base_Stop_IT(&htim15);
			 }
	}
  /* USER CODE END TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
