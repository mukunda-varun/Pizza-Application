/*
 * Android_Interface.c
 *
 *  Created on: 14-Oct-2020
 *      Author: Supriya
 */

#include "main.h"
#include "Android_Interface.h"
#include "cmsis_os2.h"
#include "usb_device.h"
#include "dc_motor_run.h"

/*Extern Timers*/

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


/*Extern Timers*/

/*Extern Variables*/

extern uint32_t time_measure;
extern uint8_t ref_msg;
extern uint8_t local_action;
extern uint8_t bladeDutyCycle;
extern uint8_t init_completed;
extern uint8_t pizza_quantity;
//extern volatile uint16_t Kneader_movement_previous_xx_mm;
//extern volatile uint16_t ejcter_previous_xx_mm;
extern uint32_t water_pulse_count;
extern char indata[64];
extern uint8_t process_abort;
extern uint8_t process_start;
extern uint32_t data_len;
extern int8_t status;
extern uint8_t data_id;
extern volatile uint32_t currentBladePWM,previousBladePWM;
extern volatile uint8_t dough_ejection_complete;
extern uint8_t stop, start1;
extern uint8_t received_data[64],Data_Reciption;
extern uint32_t received_data_size;
extern uint32_t receive_total;
extern void CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

extern void parameterValueAssignment(void);
extern uint8_t kneading_state_android, press_state_android,kneading_process_percent,press_process_percent;
extern uint8_t complete_process_done;
extern volatile uint32_t Water_current_weight , Oil_current_weight, Flour_current_weight , Flour_wt, Water_wt, Oil_wt;
extern volatile uint32_t bakingValue,bakingValue_left_counter,bakingValue_flag;

uint16_t Total_Flour_qty=0,Total_Water_qty=0,Total_oil_qty=0, Total_dough_possible_flour = 0, Total_dough_possible_water = 0, Total_dough_possible_oil = 0;
uint16_t actualDoughPossibleOil = 0, actualDoughPossibleWater = 0, actualDoughPossibleFlour = 0;
uint8_t msg[64];

extern Pizza_setting_t  Pizza_setting;
extern Ingredient_mixing_init_t Ingredient_mixing_parameters;
extern Kneading_param_t Kneading_parameters;
extern Coating_Rounding_t Coating_Rounding;
extern Dough_Pizza_Ejec_t Dough_Pizza_Eject;
extern Pressing_Baking_t Pressing_Baking;
extern Hardware_component_setting1 HW_Setting1;
extern Offset_Values_struct Offset_Values_Setting;
extern Common_machine_setting commonMCSetting;
extern calibStruct_t calibServiceMenuStruct;
press_calib_t pressCalibServiceMenuStruct;
extern stepperLeadscrew kneadLeadscrewTravel, ejectorLeadscrewTravel;
extern temperature_t kTypeTemperature;
extern loadcellStruct *flourLoadCellValue, *waterLoadCellValue, *oilLoadCellValue;
extern LPS_t LPS_struct;
extern Stepper_Motor Ejecter_Stepper_Motor, Knead_screw_Stepper_Motor, Flour_Stepper_Motor, Oil_Stepper_Motor, Water_Stepper_Motor;

/*Extern Variables*/
//extern Ingredient_mixing_init_uinon Ingredient_mixing_info;
void StartAndroidProcessingTask(void *argument)
{

	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	MX_USB_DEVICE_Init();
	for(;;)
	{
		//		status = CDC_Receive_FS(indata , &re_len);
		if(Data_Reciption)
		{

			Data_Reciption=0;
			if ( received_data[0]==0x13 && received_data[63]==0x12  )
			{

				data_id = received_data[1];
				switch(data_id)
				{
				case HANDSHAKE:
					//machine initialization and current quantity
					Send_ADR_Response(HANDSHAKE,1);
					break;

				case QUANTITY_UPDATE :
					memcpy(&Pizza_setting,&received_data[2],sizeof(Pizza_setting));
					Update_Quantity_Sufficiency();
					break;

				case  MIXING_INIT:
					memcpy(&Ingredient_mixing_parameters,&received_data[2],sizeof(Ingredient_mixing_parameters));
					Send_ADR_Response(MIXING_INIT,1);
					break;

				case KNEADING_COATING:
					memcpy(&Kneading_parameters,&received_data[2],sizeof(Kneading_parameters));
					Send_ADR_Response(KNEADING_COATING,1);
					break;

				case   DR_ID:
					memcpy(&Coating_Rounding,&received_data[2],sizeof(Coating_Rounding));
					Send_ADR_Response(DR_ID,1);
					break;

				case   DPE:
					memcpy(&Dough_Pizza_Eject,&received_data[2],sizeof(Dough_Pizza_Eject));
					Send_ADR_Response(DPE,1);
					break;

				case	   PB_ID:
					memcpy(&Pressing_Baking,&received_data[2],sizeof(Pressing_Baking));
					Send_ADR_Response(PB_ID,1);
					break;

				case CM_SETTING:
					memcpy(&commonMCSetting,&received_data[2],sizeof(commonMCSetting));
					Send_ADR_Response(CM_SETTING, 1);
					break;

				case HW_SETTING_1:
					memcpy(&HW_Setting1,&received_data[2],sizeof(HW_Setting1));
					Send_ADR_Response(HW_SETTING_1, 1);
					break;

				case HW_SETTING_2:
					memcpy(&Offset_Values_Setting,&received_data[2],sizeof(Offset_Values_Setting));
					Send_ADR_Response(HW_SETTING_2, 1);
					break;

					/* Added on 15-Feb */
				case DAILY_MAINTENANCE_STATE:
				{
					local_action = received_data[2];
					switch(local_action)
					{
						case MAINTENANCE_FLOUR_PRIMING:
							 Cleaning_state_t = flour_motor_priming_run_init;
	//							 Send_ADR_Response(MAINTENANCE_FLOUR_PRIMING,1);
							 break;
						case MAINTENANCE_WATER_PRIMING:
							 Cleaning_state_t = water_pump_priming_run_init;
	//							 Send_ADR_Response(MAINTENANCE_WATER_PRIMING,1);
							 break;
						case MAINTENANCE_WATER_CLEANING:
							 Cleaning_state_t = water_pump_cleaning_run_init;
	//							 Send_ADR_Response(MAINTENANCE_WATER_CLEANING,1);
							 break;
						case MAINTENANCE_OIL_PRIMING:
							 Cleaning_state_t = oil_pump_priming_run_init;
	//							 Send_ADR_Response(MAINTENANCE_OIL_PRIMING,1);
							 break;
						case MAINTENANCE_OIL_CLEANING:
							 Cleaning_state_t = oil_pump_cleaning_run_init;
	//							 Send_ADR_Response(MAINTENANCE_OIL_CLEANING,1);
							 break;
						case KNEADER_CLEANING_TOP:
							 Cleaning_state_t = kneader_homing_or_rear_end_init;
	//							 Send_ADR_Response(KNEADER_CLEANING_TOP,1);
							 break;
						case KNEADER_CLEANING_BTM:
							 Cleaning_state_t = kneader_bottom_end_init;
	//							 Send_ADR_Response(KNEADER_CLEANING_BTM,1);
							 break;
						case KNEADER_CLEANING_POS:
							 Cleaning_state_t = kneader_Cleaning_position_init;
	//							 Send_ADR_Response(KNEADER_CLEANING_POS,1);
							 break;
						case EJECTOR_CLEANING_START:
							 Cleaning_state_t = ejecter_homing_or_rear_end_init;
	//							 Send_ADR_Response(EJECTOR_CLEANING_START,1);
							 break;
						case EJECTOR_CLEANING_END:
							 Cleaning_state_t = ejecter_front_end_position_init;
	//							 Send_ADR_Response(EJECTOR_CLEANING_END,1);
							 break;
						case PRESS_CLEANING_TOP:
							 Cleaning_state_t = press_homing_or_revrse_init;
	//							 Send_ADR_Response(PRESS_CLEANING_TOP,1);
							 break;
						case PRESS_CLEANING_BTM:
							 Cleaning_state_t = press_bottom_limit_or_xxmm_init;
	//							 Send_ADR_Response(PRESS_CLEANING_BTM,1);
							 break;
					}
					time_measure=1;
					ref_msg  = DAILY_MAINTENANCE_STATE;
					Send_ADR_Response(ref_msg,1);
				}
					 break;
				/* Added on 15-Feb */

				 /* Added on 09-Mar */
				case SERVICE_WATER_MEASUREMENT:
					 memcpy(&calibServiceMenuStruct,&received_data[2],sizeof(calibServiceMenuStruct));
					 serviceCalibrationState = waterCalibrateStart;
					 time_measure=1;
					 ref_msg  = SERVICE_WATER_MEASUREMENT;
					 Send_ADR_Response(ref_msg,1);
					 break;
				case SERVICE_OIL_MEASUREMENT:
					 memcpy(&calibServiceMenuStruct,&received_data[2],sizeof(calibServiceMenuStruct));
					 serviceCalibrationState = oilCalibrateStart;
					 time_measure=1;
					 ref_msg  = SERVICE_OIL_MEASUREMENT;
					 Send_ADR_Response(ref_msg,1);
					 break;
				case SERVICE_FLOUR_MEASUREMENT:
					 memcpy(&calibServiceMenuStruct,&received_data[2],sizeof(calibServiceMenuStruct));
					 serviceCalibrationState = flourCalibrationStart;
					 time_measure=1;
					 ref_msg  = SERVICE_FLOUR_MEASUREMENT;
					 Send_ADR_Response(ref_msg,1);
					 break;
				case SERVICE_PRESS_MEASUREMENT:
					 memcpy(&pressCalibServiceMenuStruct,&received_data[2],sizeof(pressCalibServiceMenuStruct));
	//					 serviceCalibrationState = flourCalibrationStart;
					 time_measure=1;
					 ref_msg  = SERVICE_PRESS_MEASUREMENT;
					 Send_ADR_Response(ref_msg,1);
					 break;
				/*case MAINTENANCE_ABORT:
					 Cleaning_state_t = cleaning_abort_process;
//							 Send_ADR_Response(MAINTENANCE_ABORT,1);
					 break;*/
				case MAINTENANCE_ABORT:
					 Cleaning_state_t = cleaning_abort_process;
					 Send_ADR_Response(MAINTENANCE_ABORT,1);
					 break;
				/* Added on 09-Mar */

				case START:
					parameterValueAssignment();
					PWM_Channels_Init();
					complete_process_done = 0;
					Main_process_state = Main_Start;
					start1 = 1;
					process_start=1;
					Send_ADR_Response(START,1);
					break;
				case ABORT:
					start1 = 0;
					stop = 1;
					process_abort=1;
					process_start=0;



					/*Make all zero and stop and reset the timers and PWM channels*/
					HAL_GPIO_WritePin(DC_SSR_OP1_GPIO_Port, DC_SSR_OP1_Pin, RESET);//stop blade motor
					currentBladePWM = 0;
					bladeDutyCycle=0;
					previousBladePWM=0;
					Kneading_motor_state = Kneading_motor_init;

					HAL_TIM_Base_Stop_IT(&htim7); //dispense timer
					water_pulse_count = 0;
					Water_Stepper_Motor.rpm_counter=0;
					Water_Stepper_Motor.steps_counter=0;
					Water_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port,WATER_DISP_EN_Pin,RESET);
					HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin,RESET);


					Oil_Stepper_Motor.rpm_counter=0;
					Oil_Stepper_Motor.steps_counter=0;
					Oil_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
					HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);


					Flour_Stepper_Motor.rpm_counter=0;
					Flour_Stepper_Motor.steps_counter=0;
					Flour_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin,RESET);
					HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin,RESET);

					HAL_TIM_Base_Stop_IT(&htim14);
					Knead_screw_Stepper_Motor.rpm_counter=0;
					Knead_screw_Stepper_Motor.steps_counter=0;
					Knead_screw_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
					HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
					kneadLeadscrewTravel.previousMMTravel = 0;
//					Kneader_movement_previous_xx_mm=0;

					HAL_TIM_Base_Stop_IT(&htim15);
					Ejecter_Stepper_Motor.rpm_counter=0;
					Ejecter_Stepper_Motor.steps_counter=0;
					Ejecter_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
					HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);

					init_completed=0;
					dough_ejection_complete = 0;
					pizza_quantity = 0;
					complete_process_done = 2;//to update end process packet to android
					kneading_process_percent=0;
					press_process_percent = 0;
					machine_initialization_state = machine_initialization_idle;
					kneadingProcessState=kneading_idle;
					Press_motor_state = Press_motor_idle;
					Main_process_state = Main_Idle;
					startButton = startRead ;
					Toggle_motor_main_state = Toggle_motor_idle;
					Kneading_motor_state = Kneading_motor_idle;
					knead_screw_homing_or_top_end_limit_state = knead_screw_idle ;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
					Knead_movement_to_xx_mm_state = knead_screw_idle;
					Toggle_motor_clockwise = Toggle_motor_end;
					Toggle_motor_anticlockwise = Toggle_motor_end;
					Toggle_motor_clockwise_anticlockwise = Toggle_motor_end;
					Toggle_motor_state = Toggle_motor_end;
					Agitator_motor_state = Agitator_motor_idle;
					Press_motor_state = Press_motor_idle;
					ejecter_home_position_state = ejecter_idle;
					ejecter_movment_to_xx_mm_state  = ejecter_idle;
					ejecter_front_end_limit_position_state = ejecter_idle;
					Flour_stepper_motor_state = Flour_stepper_motor_idle;
					Oil_stepper_motor_state = Oil_stepper_motor_idle;
					Water_stepper_motor_state = Water_stepper_motor_idle;

					/*Make all zero and stop and reset the timers and PWM channels*/

					/*HAL_TIM_Base_Stop_IT(&htim7); //dispense timer
					water_pulse_count = 0;
					Water_Stepper_Motor.rpm_counter=0;
					Water_Stepper_Motor.steps_counter=0;
					Water_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port,WATER_DISP_EN_Pin,RESET);
					HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin,RESET);


					Oil_Stepper_Motor.rpm_counter=0;
					Oil_Stepper_Motor.steps_counter=0;
					Oil_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
					HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);


					Flour_Stepper_Motor.rpm_counter=0;
					Flour_Stepper_Motor.steps_counter=0;
					Flour_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin,RESET);
					HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin,RESET);

					HAL_TIM_Base_Stop_IT(&htim14);
					Knead_screw_Stepper_Motor.rpm_counter=0;
					Knead_screw_Stepper_Motor.steps_counter=0;
					Knead_screw_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
					HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
					kneadLeadscrewTravel.previousMMTravel = 0;

					HAL_TIM_Base_Stop_IT(&htim15);
					Ejecter_Stepper_Motor.rpm_counter=0;
					Ejecter_Stepper_Motor.steps_counter=0;
					Ejecter_Stepper_Motor.total_no_of_steps =0;
					HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
					HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);

					HAL_GPIO_WritePin(DC_SSR_OP1_GPIO_Port, DC_SSR_OP1_Pin, RESET); */
					PWM_Channels_Stop();
					Send_ADR_Response(ABORT,1);
					PWM_Channels_Init();
					machine_initialization_state = machine_initialization;
					break;
				case  STATUS:
					Send_ADR_Response(STATUS,1);
					break;
				}
				memset(indata,0,64);
			}
			osDelay(1000);
		}
	}

}

void  Send_ADR_Response(uint8_t id ,uint8_t data)
{
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = id;
	msg[2] = data;
	//Added on 24-03
	if(data == 1 && id == SERVICE_PRESS_MEASUREMENT)
	{
		msg[3] = (uint8_t)(LPS_struct.calibratedValueinMM * 10);
	}
	//Added on 24-03
	msg[63] = 0x12;
	CDC_Transmit_FS(msg,64);
}

void  Update_Quantity_Sufficiency(void)
{
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = 2;
	Total_Flour_qty=Pizza_setting.coatingFlourQty+Pizza_setting.flourQty1+Pizza_setting.flourQty2+Pizza_setting.flourQty3;
	Total_Water_qty=Pizza_setting.coatingWaterQty+Pizza_setting.waterQty1+Pizza_setting.waterQty2+Pizza_setting.waterQty3+Pizza_setting.waterQty4+Pizza_setting.waterQty5;
	Total_oil_qty=Pizza_setting.coatingOilQty+Pizza_setting.oilQty1+Pizza_setting.oilQty2+Pizza_setting.oilQty3;

	if(HW_Setting1.Flour_Level_0 == 0)
	{
		HW_Setting1.Flour_Level_0 = FLOUR_MIN_WT;
	}
	if(HW_Setting1.Oil_loadcell_Min_Level == 0)
	{
		HW_Setting1.Oil_loadcell_Min_Level = OIL_MIN_WT;
	}
	if(HW_Setting1.Water_loadcell_Min_Level == 0)
	{
		HW_Setting1.Water_loadcell_Min_Level = WATER_MIN_WT;
	}


	if(Total_Flour_qty > 0)
	{
		Total_dough_possible_flour = flourLoadCellValue->loadCellValueAfterTare / Total_Flour_qty;
	}
	if(Total_Water_qty > 0)
	{
		Total_dough_possible_water = waterLoadCellValue->loadCellValueAfterTare / Total_Water_qty;
	}
	if(Total_oil_qty > 0)
	{
		Total_dough_possible_oil = oilLoadCellValue->loadCellValueAfterTare / (Total_oil_qty/10);
	}
	if(Total_dough_possible_flour > 0 && (Total_dough_possible_flour > MIN_FLOUR_DOUGH_POSSIBLE) && (Total_dough_possible_flour - MIN_FLOUR_DOUGH_POSSIBLE) < 255)
	{
		if(Total_dough_possible_flour - MIN_FLOUR_DOUGH_POSSIBLE >= 30)
		{
			msg[2] = 30;				//Modified for Alpha V1, for consistency/reliablity reason on 01-03-21
		}
		else
		{
			msg[2] = (Total_dough_possible_flour - MIN_FLOUR_DOUGH_POSSIBLE);
		}
	}
	else
	{
		msg[2] = 0;
	}
	if(Total_dough_possible_water > 0 && (Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE) < 255)
	{
		if(Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE == 128)
		{
			msg[4] = 127;		//Added to avoid -128 on the android on 01-03-21
		}
		else
		{
			msg[4] = (Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE);
		}
	}
	else if(Total_dough_possible_water > 0 && (Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE) >= 255 )
	{
		msg[4] = (Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE);
		msg[5] = ((Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE) >> 8);
	}
	if(Total_dough_possible_oil > 0 && (Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE) < 255)
	{
		if(Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE == 128)
		{
			msg[6] = 127;		//Added to avoid -128 on the android on 01-03-21
		}
		else
		{
			msg[6] = (Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE);
		}
	}
	else if(Total_dough_possible_oil > 0 && (Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE) >= 255)
	{
		msg[6] = (Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE);
		msg[7] = ((Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE) >> 8);
	}
	/*Added to throw an error if total dough possible is less than the quantity given*/
	if((Total_Flour_qty > 0) && ((Total_dough_possible_flour - MIN_FLOUR_DOUGH_POSSIBLE) ) <  Pizza_setting.quantity)
	{
		msg[8] = 0;//0;//Red
	}
	else if((Total_Flour_qty > 0) && ((Total_dough_possible_flour - MIN_FLOUR_DOUGH_POSSIBLE)) >=  Pizza_setting.quantity)
	{
		msg[8] = 1;//green
	}
	if((Total_Water_qty > 0) &&( (Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE)) >=  Pizza_setting.quantity)
	{
		msg[9] = 1;////green
	}
	else if((Total_Water_qty > 0) && ((Total_dough_possible_water - MIN_WATER_DOUGH_POSSIBLE)) <  Pizza_setting.quantity)
	{
		msg[9] = 0;//0;//Red
	}
	if((Total_oil_qty > 0) &&((Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE)) <  Pizza_setting.quantity)
	{
		msg[10] = 0;//0;//red
	}
	else if((Total_oil_qty > 0) &&((Total_dough_possible_oil - MIN_OIL_DOUGH_POSSIBLE)) >=  Pizza_setting.quantity)
	{
		msg[10] = 1;//green
	}
	/*Added to throw an error if total dough possible is less than the quantity given*/
	msg[11] = (kTypeTemperature.topPanTemperature);	//Top pan temperature		Temperature_value_Sensor_top_pan
	msg[12] = 0;//(Temperature_value_Sensor_top_pan << 8);
	msg[13] = kTypeTemperature.bottomPanTemperature;//Bottom Pan Temperature	Temperature_value_Sensor_bottom_pan
	msg[14] = 0;
	CDC_Transmit_FS(msg,64);
}
void Update_end_of_process_status(void)
{
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = 0x45;
	msg[63] = 0x12;
	CDC_Transmit_FS(msg,64);
}
void  Update_Status(int i)
{
	//{0x13,10,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PA8,PA9,PA10,PA11,
	//PA12,PA13,PA14,PA15,PA16,PA17,.............,Checksum_byte,0x12}
	Total_Flour_qty=Coating_Rounding.Flour_Coating_qty+Ingredient_mixing_parameters.Flour_Qty1+Ingredient_mixing_parameters.Flour_Qty2+Ingredient_mixing_parameters.Flour_Qty3;
	Total_Water_qty=Coating_Rounding.water_coating_qty+Ingredient_mixing_parameters.Water_Qty_1+Ingredient_mixing_parameters.Water_Qty_2+Ingredient_mixing_parameters.Water_Qty_3+Ingredient_mixing_parameters.Water_Qty_4+Ingredient_mixing_parameters.Water_Qty_5;
	Total_oil_qty=Coating_Rounding.oil_coating_qty+Ingredient_mixing_parameters.Oil_Qty1+Ingredient_mixing_parameters.Oil_Qty2+Ingredient_mixing_parameters.Oil_Qty3;

	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = 10;//id
	msg[2] = pizza_quantity;										//PA1		//Pending Dough to be completed
	msg[4] = Pizza_setting.quantity;								//PA2		//Total doughs
	msg[6] = (kTypeTemperature.topPanTemperature);					//PA3	//Top pan temperature		Temperature_value_Sensor_top_pan
	msg[7] = 0;//(kTypeTemperature.topPanTemperature << 8);
	msg[8] = kTypeTemperature.bottomPanTemperature;					//PA4	//Bottom Pan Temperature	kTypeTemperature.bottomPanTemperature
	msg[9] = 0;
	msg[10] = flourLoadCellValue->loadCellValueAfterTare;			//PA5	//Flour Quantity
	msg[11] = flourLoadCellValue->loadCellValueAfterTare>>8;			//Flour Quantity for 4000gms
	msg[12] = waterLoadCellValue->loadCellValueAfterTare;			//PA6	//Water Quantity
	msg[13] = waterLoadCellValue->loadCellValueAfterTare>>8;			//Water Quantity for 4000gms
	msg[14] = oilLoadCellValue->loadCellValueAfterTare;				//PA7	//Oil Quantity
	msg[15] = (oilLoadCellValue->loadCellValueAfterTare>>8);
	msg[16] = Total_oil_qty;										//PA8	//Oil Quantity selected for Pizza
	msg[18] = Total_Water_qty;										//PA9	//Water Quantity Selected for Pizza
	if(Total_Flour_qty == 128)
	{
		msg[20] = 129;
	}
	else
	{
		msg[20] = Total_Flour_qty;									//PA10	//Flour Quantity Selected for Pizza
	}
	msg[22] = (Pressing_Baking.Thickness_Of_pizza / 10);			//PA11	//Recipe Thickness	// Added divide by 10 on 01-03-21
	msg[24] = (uint16_t)(LPS_struct.calibratedValueinMM * 10);		//PA12	//Current Thickness(Added *10 on 20-Feb for Reference on android)
	msg[26] = bakingValue_left_counter;								//PA13		//Baking time left
	msg[28] = press_process_percent;								//PA14	//Baking Process completed in percentage
	msg[28+2] = press_state_android;								//PA15		//Baking Status Ids
	msg[29+2] = 0;			//
	msg[32] = kneading_process_percent;								//PA16	//Kneading Process Completed in %
	msg[34] = kneading_state_android;								//PA17		//Kneading Process Ids
	msg[35] = 0;
	msg[36] = 0;													//PA18
	msg[38] = 0;													//PA19
	msg[63] = 0x12;
	CDC_Transmit_FS(msg,64);
}
/* USER CODE END StartTask02 */
