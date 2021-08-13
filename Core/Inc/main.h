/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LHS_Pos_Sens.h"
#include <string.h>
#include "math.h"
#include <stdlib.h>
#include "Speed_PID.h"
#include "flash.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern float getTOFValue(uint8_t slaveAddress);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

typedef struct calibStruct
{
uint16_t calibDispenseQty;//
uint16_t calibDispenseRotPerGm;//
}calibStruct_t;

typedef struct press_calib
{
	uint16_t calibPressTimeInSec;//
	uint16_t calibPressDutyCycle;//
}press_calib_t;

/*
Common Machine Settings ,Cleaning,Flour Level Sensor
{0x13,0x06,P401,,P402,,P403,,P404,,P405,,P406,,P407,,P408,,P409,,P410,P451,,P452,P453,P461,P462,P463,P464,P465,P466,P467,P468,P469,P470,...,Checksum_byte,0x12}
acknowledge/response
{0x13,0x06, 01 ,.....................,Checksum_byte,0x12}



    Hardware Component Settings
{0x13,0x07,P471,P472,P473,P474,P475,P476,P477,P478,P479,P480,P481,P482,P483,P484,P485,P486,P487,P488,P489,P490,P491,P492,P493,......,Checksum_byte,0x12}
acknowledge/response
{0x13,0x07, 01 ,.....................,Checksum_byte,0x12}
*/
/**Structures**/
typedef struct Common_machine_settings
{
uint16_t Oil_cleaning_time;//P401
uint16_t Oil_priming_time;//P402
uint16_t Water_cleaning_time;//P403
uint16_t Water_priming_time;//P404
uint16_t Flour_priming_time;////P405
uint16_t Kneader_Base_Cleaning_Position;//P406
uint16_t Idle_top_pan_temperature;////P407
uint16_t Idle_bottom_pan_temperature;//P408
uint16_t Idle_time;//P409
uint16_t Gap_between_blade_and_dough_base;//P410
uint16_t extra1;//P451
uint16_t extra2;//P452
uint16_t extra3;//P453
}Common_machine_setting;


typedef struct Hardware_components_settings1
{
uint16_t Water_Pump_Speed;//P461
uint16_t Water_Pump_Flow_Rate;//P462
uint16_t Oil_Pump_Speed;//P463
uint16_t Oil_Pump_Flow_Rate;//P464
uint16_t Flour_Motor_Speed;////P465
uint16_t Flour_Flow_rate_at_level_5;//P466
uint16_t Agitator_Motor_Duty_Cycle;////P467
uint16_t Kneading_leadscrew_motor_Speed;//P468
uint16_t Rounding_Motor_Speed;////P469
uint16_t No_Of_pulser_per_ml;//470
uint16_t Oil_loadcell_Min_Level;//P471
uint16_t Oil_loadcell_Max_Level;//P472
uint16_t Water_loadcell_Min_Level;//P473
uint16_t Water_loadcell_Max_Level;////P474
uint16_t Thickness_Reference_value;//P475
uint16_t Speed_Correction_Factor;//P476
uint16_t Top_Heater_Temp_Deviation;//P477
uint16_t Bottom_heater_Temp_Deviation;//P478
uint16_t Feed_Rate_1;//P479
uint16_t Feed_Rate_2;//P480
uint16_t Feed_Rate_3;//P481
uint16_t Feed_Rate_4;//P482
uint16_t Feed_Rate_5;//P483
uint16_t Flour_Level_0;//P484
uint16_t Flour_Level_1;//P485
uint16_t Flour_Level_2;//P486
uint16_t Flour_Level_3;//P487
uint16_t Flour_Level_4;//P488
uint16_t Flour_Level_5;//P489
}Hardware_component_setting1;

typedef struct Offset_Values_t
{
uint16_t Oil_Loadcell_Offset_Value;////P490
uint16_t Water_Loadcell_Offset_Value;//P491
uint16_t Flour_Loadcell_Offset_Value;//P492
uint16_t Process_Wattage_of_Bottom_heater;//P493
}Offset_Values_struct;

typedef struct
{
  uint16_t FlourType;
  uint16_t Dough_shape;
  uint16_t Dough_size;
  uint16_t Dough_thickness;
  uint16_t quantity;
  uint16_t baking_time;
  uint16_t waterQty1;
  uint16_t waterQty2;
  uint16_t waterQty3;
  uint16_t waterQty4;
  uint16_t waterQty5;
  uint16_t coatingWaterQty;
  uint16_t flourQty1;
  uint16_t flourQty2;
  uint16_t flourQty3;
  uint16_t coatingFlourQty;
  uint16_t oilQty1;
  uint16_t oilQty2;
  uint16_t oilQty3;
  uint16_t coatingOilQty;
}Pizza_setting_t;

typedef struct Ingredient_mixing_init
{
uint16_t Water_Qty_1;
uint16_t Water_Qty_2;
uint16_t Water_Qty_3;
uint16_t Water_Qty_4;
uint16_t Water_Qty_5;
uint16_t Flour_Qty1;
uint16_t Flour_Qty2;
uint16_t Flour_Qty3;
uint16_t Oil_Qty1;
uint16_t Oil_Qty2;
uint16_t Oil_Qty3;
uint16_t Mixing_speed_1;
uint16_t Mixing_time_1;
uint16_t Mixing_time_2;
uint16_t Mixing_time_3;
uint16_t Mixing_time_4;
uint16_t Mixing_time_5;
uint16_t Dough_Base_Step_1; //Added on 11-10-2020
uint16_t Dough_Base_Step_2;
uint16_t Dough_Base_Step_3;
uint16_t Rounding_Motor_Clock_Wise_time_1;
uint16_t Rounding_Motor_Clock_Wise_time_2;
uint16_t Rounding_Motor_Clock_Wise_time_3;
}Ingredient_mixing_init_t;


typedef struct Kneading
{
uint16_t Kneading_Speed_1;
uint16_t Kneading_Speed_2;
uint16_t Kneading_Speed_3;
uint16_t Kneading_Speed_4;
uint16_t Kneading_Speed_5;
uint16_t Kneading_Speed_6;
uint16_t Kneading_Time_1;
uint16_t Kneading_Time_2;
uint16_t Kneading_Time_3;
uint16_t Kneading_Time_4;
uint16_t Kneading_Time_5;
uint16_t Kneading_Time_6;
uint16_t Dough_Base_Step_4;
uint16_t Dough_Base_Step_5;
uint16_t Dough_Base_Step_6;
uint16_t Dough_Base_Step_7;
uint16_t Dough_Base_Step_8;
uint16_t Dough_Base_Step_9;
uint16_t Rounding_Motor_Clock_Wise_time_4;
uint16_t Rounding_Motor_Clock_Wise_time_5;
uint16_t Rounding_Motor_Clock_Wise_time_6;
uint16_t Rounding_Motor_Clock_Wise_time_7;
uint16_t Rounding_Motor_Clock_Wise_time_8;
uint16_t Rounding_Motor_Clock_Wise_time_9;
}Kneading_param_t;


typedef struct Coating_Rounding
{
uint16_t Dough_Base_Step_10;
uint16_t Coating_Speed;
uint16_t Flour_Coating_qty;
uint16_t oil_coating_qty;
uint16_t water_coating_qty;
uint16_t pre_coating_clockwise_anticlockwise_count;
uint16_t pre_coating_clockwise_time;
uint16_t pre_coating_anticlockwise_time;
uint16_t pre_coating_delay_between_shifts;
uint16_t Rouding_Clock_wise_cycles;
uint16_t Rounding_time;
uint16_t delay_between_cycles;
uint16_t Roudnig_motor_clockwise_and_anitclockwise_cycles;
uint16_t clockwise_time;
uint16_t Anti_clock_wise_time;
uint16_t delay_between_directions_shift;
}Coating_Rounding_t;

typedef struct DPE
{
uint16_t Dough_Base_Step_11;
uint16_t Dough_release_cycle_clockwise_and_anticlockwise;
uint16_t Dough_release_clockwise_time;
uint16_t Dough_releae_anit_clokcwise_time;
uint16_t Delay_between_cycles;
uint16_t Ejector_forward_speed1;
uint16_t Ejector_forward_speed_2;
uint16_t Ejector_Forward_speed_3;
uint16_t Ejector_Reverse_speed;
uint16_t Ejector_Forward_Distance_1;
uint16_t Ejector_Centre_Distance;

}Dough_Pizza_Ejec_t;


typedef struct PNB_process
{
uint16_t initial_Press_Motor_Speed;
uint16_t Intial_Press_Motor_running_time;
uint16_t Pressing_Force_Duty_cycle;
uint16_t Thickness_Of_pizza;
uint16_t Holding_Time;
uint16_t Press_Motor_reverse_duty_cycle;
uint16_t Top_Pan_Cut_off_Temp;
uint16_t Bottom_Pan_Cut_off_Temp;
uint16_t Reverse_Force_Duty_Cycle;
uint16_t Reverse_Speed_Running_Time;
}Pressing_Baking_t;

typedef struct components_settings
{
	uint16_t Water_Pump_Flow_Rate;
	uint16_t Water_Pump_Speed;
	uint16_t No_OF_pulser_per_min;
	uint16_t Oil_Pump_Flow_Rate;
	uint16_t Oil_Pump_Speed;
	uint16_t Flow_Level_0;
	uint16_t Flow_Level_1;
	uint16_t Flow_Level_2;
	uint16_t Flow_Level_3;
	uint16_t Flow_Level_4;
	uint16_t Flow_Level_5;
	uint16_t Feed_Rate_1;
	uint16_t Feed_Rate_2;
	uint16_t Feed_Rate_3;
	uint16_t Feed_Rate_4;
	uint16_t Feed_Rate_5;
	uint16_t Flour_Motor_Speed;
	uint16_t Flour_Flow_rate_at_level_5;
	uint16_t Agitator_Motor_Duty_Cycle;
	uint16_t Kneading_leadscrew_motor_Speed;
	uint16_t Rounding_Motor_Speed;
	uint16_t Thickness_Reference_value;
	uint16_t Intial_Wattage_of_Top_Heater;
	uint16_t Intial_Wattage_of_Bottom_Heater;
	uint16_t Process_Wattage_of_Top_heater;
	uint16_t Process_Wattage_of_Bottoom_heater;
	uint16_t Top_Heater_Temp_Deviation;
	uint16_t Bottom_heater_Temp_Deviation;
	uint16_t Oil_loadcell_Min_Level;
	uint16_t Oil_loadcell_Max_Level;
	uint16_t Water_loadcell_Min_Level;
	uint16_t Water_loadcell_Max_Level;
	uint16_t Speed_Correction_Factor;
}component_stting;//32

typedef struct status_parameters_t
{
	uint16_t Top_pan_temperature;
	uint16_t Bottom_pan_temperature;
	uint16_t Kneader_motor_status;
	uint16_t Kneading_screw_motor_statua;
	uint16_t Ejecter_motor_status;
	uint16_t Toggle_motor_status;
	uint16_t Press_motror_status;
}message_or_status_parameters;
/*Added on 17-10-2020*/
typedef struct Stepper_Motors
{
	uint32_t rpm;
	uint32_t frequency;
	uint32_t rpm_counter;
	uint32_t no_of_steps_per_gm_or_mm_or_ml;
	uint32_t steps_counter;
	uint32_t total_no_of_steps;
	uint32_t timer_clock;
	uint16_t Period;
	uint16_t prescaler;
	uint8_t  direction;
	float    step_angle;
}Stepper_Motor;


typedef struct report
{
uint32_t totalDoughBallMade;
uint32_t totalFlourConsumption;
uint32_t totalWaterConsumption;
uint32_t totalOilConsumption;
uint32_t totalDoughPressed;
uint32_t totalDoughPressed3mmThickness;
uint32_t totalDoughPressed4mmThickness;
uint32_t totalDoughPressed5mmThickness;
uint32_t totalDoughPressed6mmThickness;
}report_t;

typedef struct cnt
{
	uint32_t flourMaintenance;
	uint32_t waterMaintenance;
	uint32_t oilMaintenance;
	uint32_t pressTimerCnt;
	uint32_t kneadingTimerCnt;
	uint32_t ejectorTimerCnt;
	uint32_t togglingTimerCnt;
	uint32_t bakingTimerCnt;
}countVal;

/*Added on 16-Mar-2021*/

typedef struct ADC_Struct
{
	float calibratedValueinMM;
	uint32_t LPSRawADCValue;
}LPS_t;

typedef struct temp
{
	uint16_t topPanTemperature;
	uint16_t bottomPanTemperature;
}temperature_t;

typedef struct proxi
{
	uint8_t pressTopPositionLimit : 1;
	uint8_t leadscrewTopPositionLimit : 1;
	uint8_t leadscrewBottomPositionLimit : 1;
	uint8_t ejectorStartPositionLimit : 1;
	uint8_t ejectorFrontEndPositionLimit : 1;
	uint8_t bottomDoorProxLimit : 1;
	uint8_t topDoorProxLimit : 1;
	uint8_t ejectorMidPositionLimit : 1;
	uint8_t flourConnectorPresence : 1;
}proximity;

typedef struct proxiCnt
{
	uint32_t pressProxCnt;
	uint32_t ejectorStartProxCnt;
	uint32_t ejectorEndProxCnt;
	uint32_t leadBtmProxCnt;
	uint32_t leadTopProxCnt;
	uint32_t bottomDoorProxCnt;
	uint32_t topDoorProxCnt;
}proxCnt;

typedef struct leadscrew
{
	uint16_t newMMTravel;
	uint16_t previousMMTravel;
	uint16_t maximumMMTravel;
}stepperLeadscrew;

typedef struct loadcellStruct_t
{
	uint32_t actualLoadCellValue;		//Current_weight
	uint32_t loadCellValueAfterTare;	//_wt
	uint32_t loadCellMaskValue;			//wtr_value
	uint8_t loadCellGainSetValue;		//wtr
}loadcellStruct;

typedef struct errCnt
{
	uint32_t ejectorErrorCnt;
	uint32_t leadscrewErrorCnt;
	uint32_t waterDispense;
	uint32_t flourConnectorErrorCnt;
}errCnt_t;

typedef struct errRty
{
	uint8_t ejector;
	uint8_t leadscrew;
}errRetry_t;

typedef struct errState
{
	uint8_t kneaderLeadscrewTop : 1;
	uint8_t kneaderLeadscrewBottom : 1;
	uint8_t ejectorLeadscrewStart : 1;
	uint8_t ejectorLeadscrewEnd : 1;
	uint8_t pressBottom : 1;
	uint8_t pressTopPos : 1;
	uint8_t waterFlow : 1;
	uint8_t flourConnector : 1;
	uint8_t bottomDoorError : 1;
	uint8_t topDoorError : 1;
}errorState_t;

/*Added on 16-Mar-2021*/

typedef	struct feedrate
{
	float feedrate1;
	float feedrate2;
	float feedrate3;
	float feedrate4;
	float feedrate5;
}feedrate_t;
/**Structures**/

/*typedef struct HW_Machine_Settings
{
	uint16_t ;
	uint16_t ;
	uint16_t ;
	uint16_t ;
	uint16_t ;
	uint16_t leadScrewSpeed;
}Hardware_Machine_settings_t;*/

/**ENUMS**/
typedef enum Motor_dir{
	ANTICLOCKWISE=1,
	CLOCKWISE,
}Motor_Direction;
Motor_Direction Stepper_Dir,Ejecter_Direction, DC_Motor_Dir, Toggling_Motor_Dir,leadscrewMotorKneadingDirection;

typedef enum
{
	Agitator_motor_idle,
	Agitator_init,
	Agitator_motor_start,
	Agitator_motor_stop,
	Agitator_motor_completed
}Agitator_motor_state_t;
Agitator_motor_state_t Agitator_motor_state;

typedef enum{
	Press_motor_idle,
	Press_motor_init,
	Press_motor_homing_init,
//	Press_motor_reverse,
	Press_motor_PWM_wait,
	Press_motor_De_acceleration_down,
	Press_motor_acceleration_down,
	Press_motor_acceleration_up,
	press_till_xx_mm_thickness,
	press_till_xx_mm_thickness_wait,
	Press_motor_homing,
	Press_motor_reverse_wait,
	Press_motor_reverse_Force_Slow,
	Press_motor_stop,
	Press_motor_completed
}Press_motor_state_t;
Press_motor_state_t Press_motor_state, Press_motor_cleaning_bottom, Press_motor_service_calibration;

enum{
	DETECTED=0,
	NOT_DETECTED ,

};
typedef	enum{
	machine_initialization_idle,
	machine_initialization,
	machine_liquid_priming,
	press_motor_homing_init,
	press_motor_homing_init_wait,
	ejecter_or_Knead_screw_init,
	ejecter_homing_init,
	ejecter_homing_wait,
	ejecter_front_end_init,
	ejecter_front_end_wait,
	Knead_screw_top_end_or_homing_init,
	knead_screw_top_end_or_homing_wait,
	Knead_screw_bottom_end_init,
	knead_screw_bottom_end_wait,
	water_priming_reverse_start,
	water_priming_reverse_wait_to_complete,
	oil_priming_reverse_start,
	oil_priming_reverse_wait_to_complete,
	water_priming_forward_start,
	water_priming_forward_wait_to_complete,
	oil_priming_forward_start,
	oil_priming_forward_wait_to_complete,
	machine_initialization_complete,
}machine_initialization_state_t;
machine_initialization_state_t machine_initialization_state;

typedef enum{
	priming_idle1,
	water_priming_reverse_start1,
	water_priming_reverse_wait_to_complete1,
	oil_priming_reverse_start1,
	oil_priming_reverse_wait_to_complete1,
	water_priming_forward_start1,
	water_priming_forward_wait_to_complete1,
	oil_priming_forward_start1,
	oil_priming_forward_wait_to_complete1,
	priming_completed
}priming_state_t;
priming_state_t priming_state;

typedef enum{
	Kneading_motor_idle,
	Kneading_motor_init,
	kneadingAccelerate,
	kneadingDecellerate,
	Kneading_motor_start,
	Kneading_motor_wait,
	Kneading_motor_stop,
	Kneading_motor_completed
}Kneading_motor_state_t;
Kneading_motor_state_t Kneading_motor_state;

typedef enum{
	knead_screw_idle,
	knead_screw_start,
	knead_scerw_acceration,
	knead_screw_wait_to_complete,
	knead_screw_completed
}knead_screw_homing_or_top_end_limit_state_t;
knead_screw_homing_or_top_end_limit_state_t knead_screw_homing_or_top_end_limit_state,knead_screw_rear_end_or_bottom_end_limit_state,Knead_movement_to_xx_mm_state;



typedef enum{
	Toggle_motor_idle,
	Toggle_clockwise,
	Toggle_anticlockwise,
	Toggle_clockwise_anticlockwise,
	Toggle_motor_completed
}Toggle_motor_main_state_t;
Toggle_motor_main_state_t Toggle_motor_main_state;


typedef enum{
Toggle_loop_check,
Toggle_motor_clockwise_on,
Toggle_motor_clockwise_on_wait,
Toggle_motor_anti_clockwise_on,
Toggle_motor_anti_clockwise_on_wait,
Toggle_motor_clockwise_offtime_wait,
Toggle_motor_anticlockwise_offtime_wait,
Toggle_motor_acceleration_clockwise,
Toggle_motor_acceleration_wait,
Toggle_motor_acceleration_anti_clockwise,
Toggle_motor_decceleration_clockwise,
Toggle_motor_decceleration_anti_clockwise,
Toggle_motor_end,
}Toggle_motor_state_t;
Toggle_motor_state_t Toggle_motor_clockwise, Toggle_motor_anticlockwise, Toggle_motor_clockwise_anticlockwise,Toggle_motor_state;

typedef enum
{
	LoadCell_IDLE,
	Oil_Loadcell,
	Flour_loadcell,
	Water_Loadcell
}Loadcell_Read_state_t;
Loadcell_Read_state_t Loadcell_Read_state;

typedef enum{
	Usb_Idle,
	Machine_Init,
	Flour_dispensing,
	Oil_dispensing,
	Water_dispensing,
	Kneading_IS_Started,
	Pressing_is_started,
}Usb_message_state_t;
Usb_message_state_t Usb_message_state;

typedef enum{
	ADC2_INIT,
	ADC2_IDLE,
	ADC2_READ_DATA
}Read_ADC2_Channel_state_t;
Read_ADC2_Channel_state_t Read_ADC2_Channel_state;
#define ADC_CONVERTED_DATA_BUFFER_SIZE  8
enum{
	IC_IDLE,
	IC_DONE,
};
typedef enum
{
	startRead,
	startDetected,
	startWaitForComplete,
	startKneadingTopPos,
	startIdle,
}buttonState;
buttonState startButton;
typedef enum
{
	Main_Idle,
	Main_Start,
	Main_Init,
	Main_knead,
	Main_Eject_doughball_to_start_of_pan,
	Main_Eject_doughball_start_to_center,
	Main_Ejector_to_home_position,
	Main_press,
	Main_Baking_Wait,
	Main_Eject,
	Main_Eject_Wait,
	Main_completed
}Main_process_state_t;
Main_process_state_t Main_process_state;
typedef enum{
	ejectorPressIdle,
	ejectorToStartofPan,
	ejectorToCenterofPan,
	ejectorHomeFromCenter,
	ejectorHomeWait,
	pressProcess,
	pressBakingTime,
	ejectorPressComplete,
}ejectPressBaking;
ejectPressBaking ejectCenterPressBakeProcessState;
typedef enum
{
	kneading_idle,
	kneading_top_proximity_check_init,
	kneading_top_proximity_check,
	kneading_start_process,
	kneading_leadscrew_down_for_blade_and_jar_gap,
	kneading_start_blade_motor_at_first_rpm,

	kneading_dispense_flour_qty1,
	kneading_dispense_water_qty1,
	kneading_dispense_oil_qty1,
	kneading_dispense_wait_to_oil_qty1_complete,
	kneading_mixing_time1,
	kneading_leadscrew_step1,
	kneading_rounding_clockwise1_time,

	kneading_dispense_flour_qty2,
	kneading_dispense_water_qty2,
	kneading_dispense_oil_qty2,
	kneading_dispense_wait_to_oil_qty2_complete,
	kneading_mixing_time2,
	kneading_leadscrew_step2,
	kneading_rounding_clockwise2_time,

	kneading_dispense_flour_qty3,
	kneading_dispense_water_qty3,
	kneading_dispense_oil_qty3,
	kneading_dispense_wait_to_oil_qty3_complete,
	kneading_mixing_time3,
	kneading_leadscrew_step3,
	kneading_rounding_clockwise3_time,

	kneading_dispense_water_qty4,
	kneading_mixing_time4,
	kneading_dispense_water_qty5,
	kneading_mixing_time5,

	kneading_leadscrew_step4,
	kneading_start_blade_motor_at_second_rpm,
	kneading_blade_motor_first_mixing_time,
	kneading_start_blade_motor_at_third_rpm,
	kneading_leadscrew_step5,
	kneading_rounding_clockwise1,
	kneading_rounding_clockwise1_wait_to_complete,
	kneading_blade_motor_second_mixing_time,
	kneading_leadscrew_step6,
	kneading_rounding_clockwise2,
	kneading_rounding_clockwise2_wait_to_complete,
	kneading_start_blade_motor_at_fourth_rpm,
	kneading_blade_motor_third_mixing_time,
	kneading_start_blade_motor_at_fifth_rpm,
	kneading_leadscrew_step7,
	kneading_rounding_clockwise3,
	kneading_rounding_clockwise3_wait_to_complete,
	kneading_blade_motor_fourth_mixing_time,
	kneading_leadscrew_step8,
	kneading_rounding_clockwise4,
	kneading_rounding_clockwise4_wait_to_complete,
	kneading_start_blade_motor_at_sixth_rpm,
	kneading_blade_motor_fifth_mixing_time,
	kneading_start_blade_motor_at_seventh_rpm,
	kneading_leadscrew_step9,
	kneading_rounding_clockwise5,
	kneading_rounding_clockwise5_wait_to_complete,
	kneading_blade_motor_sixth_mixing_time,
	kneading_leadscrew_step10,
	kneading_rounding_clockwise6,
	kneading_rounding_clockwise6_wait_to_complete,

	kneading_pre_coating_rounding_clockwise,
	kneading_pre_coating_rounding_clockwise_wait_to_complete,


	kneading_round_coating_speed_rpm,
	kneading_round_flour_coating,
	kneading_round_water_coating,
	kneading_round_oil_coating,
	kneading_blade_motor_off_wait,

	kneading_rounding_clockwise_count,
	kneading_rounding_clockwise_anticlockwise_count_wait_to_complete,


	kneading_dough_release_start,
	kneading_dough_release_wait_to_complete,
//	kneading_dough_release_cycle,
	kneading_movement_x_mm_above_base_speed_reduce_wait_to_complete,
	kneading_movement_slow_1mm_above_base_wait_to_complete,
	kneading_complete,
}kneadingStates;
kneadingStates kneadingProcessState;

typedef enum
{
	doughReleaseIdle,
	doughReleaseStart,
//	doughReleasePositionWait,
	doughReleaseRounding,
	doughRelaseRoundingWaitToComplete,
	doughReleaseComplete,
}doughRelease;
doughRelease doughReleaseProcess;

typedef enum
{
	ejecter_idle,
	ejecter_start,
	ejecter_acceleration,
	ejecter_wait_to_complete,
	ejecter_completed
}Ejecter_position_control_t;
Ejecter_position_control_t ejecter_front_end_limit_position_state,ejecter_home_position_state,ejecter_movment_to_xx_mm_state;

typedef enum
{
	Oil_stepper_motor_idle,
	Oil_stepper_motor_init,
	Oil_stepper_motor_start,
	oil_stepper_motor_acceleration,
	Oil_stepper_motor_wait_to_complete,
	Oil_stepper_motor_completed,
}Oil_stepper_motor_t;
Oil_stepper_motor_t Oil_stepper_motor_state, Oil_stepper_maintenance_state;

typedef enum
{
	Flour_stepper_motor_idle,
	Flour_stepper_motor_init,
	Flour_stepper_motor_start,
	Flour_stepper_motor_acceleration,
	Flour_stepper_motor_wait_to_complete,
	Flour_stepper_motor_completed,
}Flour_stepper_motor_t;
Flour_stepper_motor_t Flour_stepper_motor_state, Flour_stepper_maintenance_state;

typedef enum
{
	Water_stepper_motor_idle,
	Water_stepper_motor_init,
	Water_stepper_motor_start,
	Water_stepper_motor_acceleration,
	Water_stepper_motor_wait_to_complete,
	Water_stepper_motor_completed,
}Water_stepper_motor_t;
Water_stepper_motor_t Water_stepper_motor_state, Water_stepper_maintenance_state;


typedef enum
{
	Idle_Dispense,
	flourDispense,
	waterDispense,
	oilDispense,
}dispenseState;
dispenseState dispenseStateForInterrupt;

enum{
	Push_button,
	trigger_1st_stage,
	first_stage_waiting,
	trigger_2nd_stage,
	second_stage_waiting,
	trigger_3rd_stage,
	third_stage_waiting,
	completed_flour_dispense
}flour_dispense_for_testing_t;

typedef enum
{
	toggleMotorClockwiseTest = 1,
	toggleMotorAnticlockwiseTest,
	pressMotorCycleTest,
	kneadingMotorTest,
	kneaderLeadscrewBottomPositionTest,
	kneaderLeadscrewTopPositionTest,
	ejectorFrontEndPositionTest,
	ejectorHomingPositionTest,
	waterStepperMotorTest,
	oilStepperMotorTest,
	flourStepperMotorTest,
	waitToComplete,
	noTest,
}testBoard;
testBoard moduleTest;

/**Added on 18-01-2021**/
typedef enum{
	waterCheck = 1,
	waterErrorCheck,
	flourCheck,
	flourErrorCheck,
	ejectorCheck,
	ejectorErrorCheck,
	leadscrewCheck,
	leadscrewErrorCheck,
	pressCheck,
	pressErrorCheck,
	errorState,
	dailyMaintenanceComplete,
	idleState,
}cleaningmenu;
cleaningmenu dailyMaintenanceMenu;

typedef enum{
	ejectorError =  1,
	leadscrewError,
	flourError,
	waterError,
	pressError,
}dailyMaintenanceError;
dailyMaintenanceError maintenanceError;
/**Added on 18-01-2021**/

/*added on 03-02-2021*/
typedef enum{
	Dialy_Maitanance_Idle=1,
	Oil_Priming_init,
	Oil_Priming_Wait,
	Flour_Priming_init,
	Flour_Priming_Wait,
	Water_Priming_init,
	Water_Priming_Wait,
}Daily_Maintanance;
Daily_Maintanance Daily_Maintanance_State_t;

typedef enum{
	priming_idle,
	reverse_priming_init,
	reverse_priming_init_wait,
	Forward_priming_init,
	Forward_priming_init_wait,
	priming_completed_t,
}ingredients_priming_state_t;
ingredients_priming_state_t water_priming_t,flour_priming_t,oil_priming_t;

typedef enum{
	calibrationIdle,
	flourCalibrationStart,
	flourCalibrationWaitToComplete,
	oilCalibrateStart,
	oilCalibrateWaitToComplete,
	waterCalibrateStart,
	waterCalibrateWaitToComplete,
	serviceCalibrateComplete,
	serviceCalibrationAbort,
}calibMenu_t;
calibMenu_t serviceCalibrationState;
typedef enum{
	cleaning_idle,
	kneader_homing_or_rear_end_init,
	kneader_home_waiting_to_ejecter_home_position,
	kneader_home_waiting_to_Press_home_position,
	kneader_homing_or_rear_end_wait,
	kneader_bottom_end_init,
	kneader_bottom_waiting_to_ejecter_home_position,
	kneader_bottom_waiting_to_Press_home_position,
	kneader_bottom_end_wait,
	kneader_Cleaning_position_init,
	kneader_clean_waiting_to_ejecter_home_position,
    kneader_clean_waiting_to_Press_home_position,
	kneader_Cleaning_home_position_wait,
	kneader_Cleaning_position_wait,
	ejecter_homing_or_rear_end_init,
	ejecter_home_waiting_to_Press_home_position,
	ejecter_home_waiting_to_kneader_home_position,
	ejecter_homing_or_rear_end_wait,
	ejecter_front_end_position_init,
	ejecter_front_waiting_to_Press_home_position,
    ejecter_front_waiting_to_kneader_home_position,
	ejecter_front_end_position_wait,
    water_pump_cleaning_run_init,
	water_pump_cleaning_run_wait,
    water_pump_priming_run_init,
	water_pump_priming_run_wait,
    oil_pump_cleaning_run_init,
	oil_pump_cleaning_run_wait,
    oil_pump_priming_run_init,
	oil_pump_priming_run_wait,
    flour_motor_cleaning_run_init,
	flour_motor_cleaning_run_wait,
    flour_motor_priming_run_init,
	flour_motor_priming_run_wait,
	press_homing_or_revrse_init,
	press_homing_or_revrse_wait,
	press_home_waiting_to_ejecter_home_position,
	press_bottom_limit_or_xxmm_init,
	press_bottom_waiting_to_ejecter_home_position,
	press_bottom_limit_or_xxmm_wait,
	cleaning_abort_process,
	cleaning_completed,
}Cleaning_state;
Cleaning_state Cleaning_state_t;
/**ENUMS**/
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef struct{
	uint16_t stepPerMMRotation;
}Motor;
Motor ejectorMotor, leadScrewMotor;

/**********Macros Value**********/

/**********************/
#define		STEPPER_TIMER_CLOCK							60000000
#define		STEPPER_STEP								0.9
#define		FLOUR_STEPPER_STEP							1.8
#define		STEPPER_TIMER_PERIOD						1000
#define		STEPS_PER_ROTATION_FULL_STEP				200
#define		STEPS_PER_ROTATION_HALF_STEP				400
/**********************/

/*******Compiler Commenting*******/

//VTOR VALUE FOR OTA Flash    0x00100000UL
//androidParameter = 0; for default Value & androidParameter = 1; for android Value to be taken from android
#define		androidParameter				1
//bladePWM = 0; for SSR control & bladePWM = 1; for android Value to be taken for Motor control
#define		bladePWM						1
//doughReleaseEnable = 0; for dough Release functionality to disable & doughReleaseEnable = 1; for dough Release functionality to enable
#define		doughReleaseEnable				1

#define		CYTRON_DRV						1				//2 = VNH7070, 1 = CYTRON, 0 = BTN7960

#define		MACHINE_1						0				//1 for Alpha M1 & 0 for Alpha M2

#define		KNEADING_TIME_REDUCE			1				//Kneading Time Reducing Macro, 1 = Enable reduction ; 0 = Disable Reduction

#define		BOARD_V2_0						1

#define		FLOUR_PRESENCE_SENSOR			0

#define		COMPLETE_PROCESS				1				//0->Disable  or 1->Enable

#define 	INTERLOCK_EN					1
/*******Compiler Commenting*******/

/**Blade Values**/
#define 	MAX_SPEED_KNEADER 							360				//MAX SPEED OF KNEADER(BLADE) MOTOR
#define 	MAX_PWM_KNEADER								100				//MAX PWM VALUE OF KNEADER
#define 	SPEED_BAND_LIMIT							10				//Speed range
#define		SPEED_CONVERTER_VALUE					((float)MAX_PWM_KNEADER / (float)MAX_SPEED_KNEADER)		//
#define		KNEADER_LEADSCREW_MULTIPLIER				2				//Half Step multiply by 2
#define 	KNEADER_LEADSCREW_STEPS_PER_MM			(80 * KNEADER_LEADSCREW_MULTIPLIER)		//80 steps per mm
#define		KNEADER_MAXIMUM_LENGTH_IN_MM				280
#define		KNEADER_LEAD_SLOW_DISTANCE_ABV_PLATE		10				//x mm above press
#define		BLADE_MAX_ACCE_DECCELERATE_CNT				8
/**Blade Values**/

/**Ejector Values**/
#define		EJECTOR_MAXIMUM_LENGTH_IN_MM				550
#define		EJECTOR_LEADSCREW_MULTIPLIER				2				//Half Step multiply by 2
#define		EJECTOR_LEADSCREW_STEPS_PER_MM			(33 * EJECTOR_LEADSCREW_MULTIPLIER)		//16.5 * 2
/**Ejector Values**/

/**Default Values**/
#define		STEPPER_MOTOR_DEFAULT_RPM					300				//RPM
#define		OIL_STEPPER_DEFAULT_RPM						200
#define		WATER_STEPPER_DEFAULT_RPM					200
#define		FLOUR_STEPPER_DEFAULT_RPM					100
#define		PRESS_REVERSE_RUNNING_TIME_MS			    2000
#define		PRESS_REVERSE_DECELLERATION_PWM				(float)30.0
#define		ROUNDING_DEFAULT_PWM_VALUE					(float)80.0
#define		PROXIMITY_DEBOUNCE_CNT						25

/*Error Related Values*/
#define		EJECTOR_ERROR_TIME							7000
#define		EJECTOR_RETRY_MAX_COUNT						3
#define		KNEADER_LEADSCREW_ERROR_TIME				9000
#define		KNEADER_LEADSCREW_RETRY_MAX_COUNT			3
/*Error Related Values*/
/**Default Values**/

/**Toggle Values**/
#define		TOGGLE_DEFAULT_OFFTIME_MS					100
#define		TOGGLE_MAX_ACCE_DECCELERATE_CNT				7
/**Toggle Values**/


/**Press Values**/
#define		PRESS_REVERSE_FORCE_TIME_MS					20000
#define		PRESS_TOP_POS_ERROR_TIME_MS					25000
#define		PRESS_REVERSE_FORCE_DIST_MM					(float)7.0
#define		PRESS_MAX_ACCE_DECCELERATE_CNT				8
#define		LPS_VALUE_DEBOUNCE_COUNT					10
#define		LPS_CALBRATE_MAX_MM							(float)15.0
#define		LPS_CALIBRATE_MAX_ADC						2847
#define		LPS_CALIBRATE_MIN_MM						(float)0.0
#define		LPS_CALIBRATE_MIN_ADC						360
#define		PRESS_DUTYCYCLE_CLEANING					40.0
#define		PRESS_DEFAULT_THICKNESS						3.0
#define		PRESS_STOP_OFFSET							0.3
#define		PIZZA_BASE_THICKNESS_CUTOFF_MM				(float)MAX_VAL_MM - 3.0 // - 2.5 for M2		//+0.5 for M1
/**Press Values**/



/**Dispensing Values**/
#define     FLOUR_DISPENSE_STEPS_PER_GM     			88 //90*2//111//90		//For for half step
#define		FLOUR_LOADCELL_WEIGHT_OFFSET				9250//10700//10030//13900//10700(Alpha-M2)
/*Flour dispense multiplication factor*/
#define		FLOUR_DISPENSE_STG1_FACTOR1					1.04
#define		FLOUR_DISPENSE_STG1_FACTOR2					1.06
#define		FLOUR_DISPENSE_STG1_FACTOR3					1.13//1.10//1.20//1.10
#define		FLOUR_DISPENSE_STG1_FACTOR4					1.30//1.40//1.14
#define		FLOUR_DISPENSE_STG1_FACTOR5					1.45//1.50//1.14


#define		FLOUR_DISPENSE_STG2_FACTOR1					1.05
#define		FLOUR_DISPENSE_STG2_FACTOR2					1.08
#define		FLOUR_DISPENSE_STG2_FACTOR3					1.15//1.13//1.23//1.13
#define		FLOUR_DISPENSE_STG2_FACTOR4					1.40//1.43//1.20
#define		FLOUR_DISPENSE_STG2_FACTOR5					1.50//1.55//1.20

#define		FLOUR_DISPENSE_STG3_FACTOR1					1.05
#define		FLOUR_DISPENSE_STG3_FACTOR2					1.08
#define		FLOUR_DISPENSE_STG3_FACTOR3					1.15//1.13//1.23//1.13
#define		FLOUR_DISPENSE_STG3_FACTOR4					1.40//1.43//1.20
#define		FLOUR_DISPENSE_STG3_FACTOR5					1.50//1.55//1.20

#define		FLOUR_DISPENSE_LEVEL1						6000
#define		FLOUR_DISPENSE_LEVEL2						4000
#define		FLOUR_DISPENSE_LEVEL3						3200
#define		FLOUR_DISPENSE_LEVEL4						2700
#define		FLOUR_DISPENSE_LEVEL5						1700
#define		FLOUR_DISPENSE_LEVEL6						800
/*Flour dispense multiplication factor*/
#define     OIL_DISPENSE_STEPS_PER_ML       			760//690	//For actual Setup
#define		OIL_LOADCELL_WEIGHT_OFFSET					650
#define		OIL_PRIMING_REVERSE_STEPS					7000
#define		OIL_PRIMING_FORWARD_STEPS					5000

#define     WATER_LOADCELL_WEIGHT_OFFSET    			2480//1550
#define		WATER_DISPENSE_STEPS_PER_ML					323
#define 	WATER_DISPENSE_STEPS_TOLERANCE				5
#define		WATER_PULSE_VALUE						    HW_Setting1.No_Of_pulser_per_ml//((float)1600/ (float)760)			//1600 pulses for 760 ml of water per ml pulse value
#define		WATER_PRIMING_REVERSE_STEPS					7000
#define		WATER_PRIMING_FORWARD_STEPS					5000
/**Dispensing Values**/
#define		MIN_FLOUR_WEIGHT							HW_Setting1.Flour_Level_0		//2700
#define		MIN_WATER_WEIGHT							HW_Setting1.Water_loadcell_Min_Level //1230
#define		MIN_OIL_WEIGHT								HW_Setting1.Oil_loadcell_Min_Level  //90

#define		OIL_MIN_WT									90
#define		WATER_MIN_WT								1230
#define		FLOUR_MIN_WT								2700

#define		MAX_FLOUR_WEIGHT							8000
#define		MAX_WATER_WEIGHT							5500
#define		MAX_OIL_WEIGHT								1200

#define		MIN_FLOUR_DOUGH_POSSIBLE					(MIN_FLOUR_WEIGHT / Total_Flour_qty)//20
#define		MIN_WATER_DOUGH_POSSIBLE					(MIN_WATER_WEIGHT / Total_Water_qty)//15
#define		MIN_OIL_DOUGH_POSSIBLE						(MIN_OIL_WEIGHT / Total_oil_qty)//20

#define  	PIZZA_BALL_MODE								1
#define		PIZZA_BASE_MODE								0

#define		TEMPERATURE_CUTOFF_VALUE					12

#define		SSR_OFF_CUTOFF								5
/**********Macros Value**********/

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void parameterValueAssignment(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAX_SPI4_SCK_Pin GPIO_PIN_2
#define MAX_SPI4_SCK_GPIO_Port GPIOE
#define MAX_SPI4_CS1_Pin GPIO_PIN_3
#define MAX_SPI4_CS1_GPIO_Port GPIOE
#define MAX_SPI4_CS2_Pin GPIO_PIN_4
#define MAX_SPI4_CS2_GPIO_Port GPIOE
#define MAX_SPI4_MISO_Pin GPIO_PIN_5
#define MAX_SPI4_MISO_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define TOF_I2C2_SDA_Pin GPIO_PIN_0
#define TOF_I2C2_SDA_GPIO_Port GPIOF
#define TOF_I2C2_SCL_Pin GPIO_PIN_1
#define TOF_I2C2_SCL_GPIO_Port GPIOF
#define DC_SSR_OP1_Pin GPIO_PIN_2
#define DC_SSR_OP1_GPIO_Port GPIOF
#define SSR_TOP_PAN_Pin GPIO_PIN_4
#define SSR_TOP_PAN_GPIO_Port GPIOF
#define SSR_BOTTOM_PAN_Pin GPIO_PIN_5
#define SSR_BOTTOM_PAN_GPIO_Port GPIOF
#define SPEED_MEAS_Pin GPIO_PIN_6
#define SPEED_MEAS_GPIO_Port GPIOF
#define SPEED_DIR_Pin GPIO_PIN_7
#define SPEED_DIR_GPIO_Port GPIOF
#define AGITATOR_MTR_PWM_Pin GPIO_PIN_8
#define AGITATOR_MTR_PWM_GPIO_Port GPIOF
#define AGITATOR_MTR_DIR_Pin GPIO_PIN_9
#define AGITATOR_MTR_DIR_GPIO_Port GPIOF
#define AGITATOR_MTR_FAULT_Pin GPIO_PIN_10
#define AGITATOR_MTR_FAULT_GPIO_Port GPIOF
#define FLOW_SENSOR_PIN_Pin GPIO_PIN_1
#define FLOW_SENSOR_PIN_GPIO_Port GPIOC
#define FLOW_SENSOR_PIN_EXTI_IRQn EXTI1_IRQn
#define EMERGENCY_STOP_Pin GPIO_PIN_2
#define EMERGENCY_STOP_GPIO_Port GPIOC
#define SPARE_DRV_MTR_DIR_Pin GPIO_PIN_2
#define SPARE_DRV_MTR_DIR_GPIO_Port GPIOA
#define SPARE_DRV_MTR_FAULT_Pin GPIO_PIN_3
#define SPARE_DRV_MTR_FAULT_GPIO_Port GPIOA
#define SPARE_DAC1_Pin GPIO_PIN_4
#define SPARE_DAC1_GPIO_Port GPIOA
#define SPARE_DAC2_Pin GPIO_PIN_5
#define SPARE_DAC2_GPIO_Port GPIOA
#define PRESS_MTR_LSENS_Pin GPIO_PIN_6
#define PRESS_MTR_LSENS_GPIO_Port GPIOA
#define PRESS_MTR_RSENS_Pin GPIO_PIN_7
#define PRESS_MTR_RSENS_GPIO_Port GPIOA
#define TOGGLING_MTR_LSENS_Pin GPIO_PIN_4
#define TOGGLING_MTR_LSENS_GPIO_Port GPIOC
#define TOGGLING_MTR_RSENS_Pin GPIO_PIN_5
#define TOGGLING_MTR_RSENS_GPIO_Port GPIOC
#define KNEAD_MTR_RSENS_Pin GPIO_PIN_1
#define KNEAD_MTR_RSENS_GPIO_Port GPIOB
#define SPARE_MTR_LSENS_Pin GPIO_PIN_11
#define SPARE_MTR_LSENS_GPIO_Port GPIOF
#define SPARE_MTR_RSENS_Pin GPIO_PIN_12
#define SPARE_MTR_RSENS_GPIO_Port GPIOF
#define LIN_POS_SN_Pin GPIO_PIN_13
#define LIN_POS_SN_GPIO_Port GPIOF
#define SHARP_SN_Pin GPIO_PIN_14
#define SHARP_SN_GPIO_Port GPIOF
#define PRESS_TOP_POS_Pin GPIO_PIN_15
#define PRESS_TOP_POS_GPIO_Port GPIOF
#define KNEAD_TOP_POS_Pin GPIO_PIN_0
#define KNEAD_TOP_POS_GPIO_Port GPIOG
#define KNEAD_BOTTOM_POS_Pin GPIO_PIN_1
#define KNEAD_BOTTOM_POS_GPIO_Port GPIOG
#define EJECTOR_START_POS_Pin GPIO_PIN_7
#define EJECTOR_START_POS_GPIO_Port GPIOE
#define EJECTOR_MID_POS_Pin GPIO_PIN_8
#define EJECTOR_MID_POS_GPIO_Port GPIOE
#define EJECTOR_END_POS_Pin GPIO_PIN_9
#define EJECTOR_END_POS_GPIO_Port GPIOE
#define SPARE_POS1_Pin GPIO_PIN_10
#define SPARE_POS1_GPIO_Port GPIOE
#define SPARE_POS2_Pin GPIO_PIN_11
#define SPARE_POS2_GPIO_Port GPIOE
#define PRESS_MTR_RPWM_Pin GPIO_PIN_13
#define PRESS_MTR_RPWM_GPIO_Port GPIOE
#define PRESS_MTR_DIR_Pin GPIO_PIN_14
#define PRESS_MTR_DIR_GPIO_Port GPIOE
#define PRESS_MTR_EN_Pin GPIO_PIN_15
#define PRESS_MTR_EN_GPIO_Port GPIOE
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_11
#define UART_TX_GPIO_Port GPIOB
#define TOGGLING_MTR_EN_Pin GPIO_PIN_13
#define TOGGLING_MTR_EN_GPIO_Port GPIOB
#define TOGGLING_MTR_RPWM_Pin GPIO_PIN_14
#define TOGGLING_MTR_RPWM_GPIO_Port GPIOB
#define TOGGLING_MTR_DIR_Pin GPIO_PIN_15
#define TOGGLING_MTR_DIR_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOD
#define USB_EN_Pin GPIO_PIN_10
#define USB_EN_GPIO_Port GPIOD
#define KNEAD_MTR_EN_Pin GPIO_PIN_11
#define KNEAD_MTR_EN_GPIO_Port GPIOD
#define KNEAD_MTR_RPWM_Pin GPIO_PIN_12
#define KNEAD_MTR_RPWM_GPIO_Port GPIOD
#define KNEAD_DIR_Pin GPIO_PIN_13
#define KNEAD_DIR_GPIO_Port GPIOD
#define OIL_LOADCELL_DATA_Pin GPIO_PIN_2
#define OIL_LOADCELL_DATA_GPIO_Port GPIOG
#define OIL_LOADCELL_CLK_Pin GPIO_PIN_3
#define OIL_LOADCELL_CLK_GPIO_Port GPIOG
#define WATER_LOADCELL_DATA_Pin GPIO_PIN_4
#define WATER_LOADCELL_DATA_GPIO_Port GPIOG
#define WATER_LOADCELL_CLK_Pin GPIO_PIN_5
#define WATER_LOADCELL_CLK_GPIO_Port GPIOG
#define SPARE_LOADCELL_DATA_Pin GPIO_PIN_6
#define SPARE_LOADCELL_DATA_GPIO_Port GPIOG
#define USB_FAULT_Pin GPIO_PIN_7
#define USB_FAULT_GPIO_Port GPIOG
#define SPARE_LOADCELL_CLK_Pin GPIO_PIN_8
#define SPARE_LOADCELL_CLK_GPIO_Port GPIOG
#define SPARE_DC_MTR_ENA_Pin GPIO_PIN_6
#define SPARE_DC_MTR_ENA_GPIO_Port GPIOC
#define SPARE_DC_MTR_ENB_Pin GPIO_PIN_7
#define SPARE_DC_MTR_ENB_GPIO_Port GPIOC
#define SPARE_MTR_PWM_Pin GPIO_PIN_8
#define SPARE_MTR_PWM_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_N_Pin GPIO_PIN_11
#define USB_N_GPIO_Port GPIOA
#define USB_P_Pin GPIO_PIN_12
#define USB_P_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define SPARE_SPI3_SCK_Pin GPIO_PIN_10
#define SPARE_SPI3_SCK_GPIO_Port GPIOC
#define SPARE_SPI3_MISO_Pin GPIO_PIN_11
#define SPARE_SPI3_MISO_GPIO_Port GPIOC
#define SPARE_SPI3_MOSI_Pin GPIO_PIN_12
#define SPARE_SPI3_MOSI_GPIO_Port GPIOC
#define SPARE_SPI3_CS1_Pin GPIO_PIN_0
#define SPARE_SPI3_CS1_GPIO_Port GPIOD
#define SPARE_SPI3_CS2_Pin GPIO_PIN_1
#define SPARE_SPI3_CS2_GPIO_Port GPIOD
#define LEADSCREW_EN_Pin GPIO_PIN_2
#define LEADSCREW_EN_GPIO_Port GPIOD
#define LEADSCREW_DIR_Pin GPIO_PIN_3
#define LEADSCREW_DIR_GPIO_Port GPIOD
#define LEADSCREW_PULSE_Pin GPIO_PIN_4
#define LEADSCREW_PULSE_GPIO_Port GPIOD
#define EJECTOR_EN_Pin GPIO_PIN_5
#define EJECTOR_EN_GPIO_Port GPIOD
#define EJECTOR_DIR_Pin GPIO_PIN_6
#define EJECTOR_DIR_GPIO_Port GPIOD
#define EJECTOR_PULSE_Pin GPIO_PIN_7
#define EJECTOR_PULSE_GPIO_Port GPIOD
#define WATER_DISP_EN_Pin GPIO_PIN_9
#define WATER_DISP_EN_GPIO_Port GPIOG
#define WATER_DISP_DIR_Pin GPIO_PIN_10
#define WATER_DISP_DIR_GPIO_Port GPIOG
#define WATER_DISP_PULSE_Pin GPIO_PIN_11
#define WATER_DISP_PULSE_GPIO_Port GPIOG
#define FLOUR_DISP_EN_Pin GPIO_PIN_12
#define FLOUR_DISP_EN_GPIO_Port GPIOG
#define FLOUR_DISP_DIR_Pin GPIO_PIN_13
#define FLOUR_DISP_DIR_GPIO_Port GPIOG
#define FLOUR_DISP_PULSE_Pin GPIO_PIN_14
#define FLOUR_DISP_PULSE_GPIO_Port GPIOG
#define SPARE_STEP_MTR_EN_Pin GPIO_PIN_15
#define SPARE_STEP_MTR_EN_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SPARE_STEP_MTR_DIR_Pin GPIO_PIN_4
#define SPARE_STEP_MTR_DIR_GPIO_Port GPIOB
#define SPARE_STEP_MTR_PULSE_Pin GPIO_PIN_5
#define SPARE_STEP_MTR_PULSE_GPIO_Port GPIOB
#define OIL_DISP_EN_Pin GPIO_PIN_6
#define OIL_DISP_EN_GPIO_Port GPIOB
#define OIL_DISP_DIR_Pin GPIO_PIN_7
#define OIL_DISP_DIR_GPIO_Port GPIOB
#define OIL_DISP_PULSE_Pin GPIO_PIN_8
#define OIL_DISP_PULSE_GPIO_Port GPIOB
#define SPARE_UART_DE_Pin GPIO_PIN_9
#define SPARE_UART_DE_GPIO_Port GPIOB
#define NUCLEO_LED_Pin GPIO_PIN_1
#define NUCLEO_LED_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
