/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hx711.h"
#include "MAX_31855.h"
#include "usbd_cdc_if.h"
#include "step_motor.h"
#include "dc_motor_run.h"
#include "ADC_READ.h"
#include "Android_Interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

uint8_t ConfArray[16];


//__attribute__((__section__(".rodata")))  const char version[10] = "MukPi-2.1";
//__attribute__((__section__(".rodata")))  const int dataval = 'B';

//const uint8_t userConfig = 0x09;
//userConfig[0] = 0x1;
//FLASH_Erase_Sector(FLASH_SECTOR_2, FLASH_BANK_2, VOLTAGE_RANGE_3);
//__attribute__((section(".userdata"),userdata)) const uint32_t value=8;
//__attribute__((section(".MY_VARS"),myvars)) const uint32_t value=8;
//__attribute__((section(".userdata"),userdata)) const uint32_t value=8;
//const uint8_t secret_key __attribute__((section(".user_data__at_0x8140000")));
//const uint8_t secret_key = 1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityHigh6,
  .stack_size = 1024 * 4
};
/* Definitions for Main_process_ta */
osThreadId_t Main_process_taHandle;
const osThreadAttr_t Main_process_ta_attributes = {
  .name = "Main_process_ta",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Load_cell */
osThreadId_t Load_cellHandle;
const osThreadAttr_t Load_cell_attributes = {
  .name = "Load_cell",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Temperature_con */
osThreadId_t Temperature_conHandle;
const osThreadAttr_t Temperature_con_attributes = {
  .name = "Temperature_con",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Android_handler */
osThreadId_t Android_handlerHandle;
const osThreadAttr_t Android_handler_attributes = {
  .name = "Android_handler",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for SpeedControl_Ta */
osThreadId_t SpeedControl_TaHandle;
const osThreadAttr_t SpeedControl_Ta_attributes = {
  .name = "SpeedControl_Ta",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Press */
osThreadId_t PressHandle;
const osThreadAttr_t Press_attributes = {
  .name = "Press",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Dispensing_Task */
osThreadId_t Dispensing_TaskHandle;
const osThreadAttr_t Dispensing_Task_attributes = {
  .name = "Dispensing_Task",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Kneading_Motor */
osThreadId_t Kneading_MotorHandle;
const osThreadAttr_t Kneading_Motor_attributes = {
  .name = "Kneading_Motor",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Adc_Process_Tas */
osThreadId_t Adc_Process_TasHandle;
const osThreadAttr_t Adc_Process_Tas_attributes = {
  .name = "Adc_Process_Tas",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Cleaning_Task */
osThreadId_t Cleaning_TaskHandle;
const osThreadAttr_t Cleaning_Task_attributes = {
  .name = "Cleaning_Task",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Service_menu */
osThreadId_t Service_menuHandle;
const osThreadAttr_t Service_menu_attributes = {
  .name = "Service_menu",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for Ejector_Task */
osThreadId_t Ejector_TaskHandle;
const osThreadAttr_t Ejector_Task_attributes = {
  .name = "Ejector_Task",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for Toggling_Task */
osThreadId_t Toggling_TaskHandle;
const osThreadAttr_t Toggling_Task_attributes = {
  .name = "Toggling_Task",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for kneading_Proces */
osThreadId_t kneading_ProcesHandle;
const osThreadAttr_t kneading_Proces_attributes = {
  .name = "kneading_Proces",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
uint8_t myQueue01Buffer[ 100 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01",
  .cb_mem = &myQueue01ControlBlock,
  .cb_size = sizeof(myQueue01ControlBlock),
  .mq_mem = &myQueue01Buffer,
  .mq_size = sizeof(myQueue01Buffer)
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* Definitions for myMutex02 */
osMutexId_t myMutex02Handle;
const osMutexAttr_t myMutex02_attributes = {
  .name = "myMutex02"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* Definitions for myBinarySem02 */
osSemaphoreId_t myBinarySem02Handle;
const osSemaphoreAttr_t myBinarySem02_attributes = {
  .name = "myBinarySem02"
};
/* Definitions for myBinarySem03 */
osSemaphoreId_t myBinarySem03Handle;
osStaticSemaphoreDef_t myBinarySem03ControlBlock;
const osSemaphoreAttr_t myBinarySem03_attributes = {
  .name = "myBinarySem03",
  .cb_mem = &myBinarySem03ControlBlock,
  .cb_size = sizeof(myBinarySem03ControlBlock),
};
/* Definitions for myCountingSem01 */
osSemaphoreId_t myCountingSem01Handle;
osStaticSemaphoreDef_t myCountingSem01ControlBlock;
const osSemaphoreAttr_t myCountingSem01_attributes = {
  .name = "myCountingSem01",
  .cb_mem = &myCountingSem01ControlBlock,
  .cb_size = sizeof(myCountingSem01ControlBlock),
};
/* USER CODE BEGIN PV */

osThreadId_t Android_Process_TaskHandle;
const osThreadAttr_t Android_Process_Tas_attributes = {
		.name = "Android_Interface_task",
		.priority = (osPriority_t) osPriorityRealtime,
		.stack_size = 1024 * 4
};


/***********Variable Declarations***********/

/*Android Structures*/

Pizza_setting_t  Pizza_setting;
Ingredient_mixing_init_t Ingredient_mixing_parameters;
Kneading_param_t Kneading_parameters;
Coating_Rounding_t Coating_Rounding;
Dough_Pizza_Ejec_t Dough_Pizza_Eject;
Pressing_Baking_t Pressing_Baking;

Hardware_component_setting1 HW_Setting1;
Offset_Values_struct Offset_Values_Setting;
Common_machine_setting commonMCSetting;

//Latest Added
calibStruct_t calibServiceMenuStruct;
press_calib_t pressCalibServiceMenuStruct;
report_t production_report_t, local_report;

/*Android Structures*/

/*Android Variables*/
uint8_t init_completed=0;
uint8_t received_data[64],Data_Reciption;
uint32_t received_data_size;
uint32_t receive_total;
/*Android Process Variables*/
uint8_t stop = 0, start1 = 0;
uint8_t complete_process_done = 2;
char indata[64];
uint8_t process_abort=0;
uint8_t process_start=0;
uint8_t re_len;
uint32_t data_len;
int8_t status;
uint8_t data_id;
uint8_t local_action;
/*Android Process Variables*/

/*Android Variables*/

/**Kneading Variables**/
volatile uint32_t bladeMotorSpeedValueFromStructure = 0;
volatile uint32_t mixingTimeValuefromStructure = 0;
float pwmValue = 0.0f;
volatile uint32_t waitCnt = 0, bladeRunCnt = 0, previousBladePWM = 0, currentBladePWM = 0;
uint8_t bladeDutyCycle = 0;
/**Kneading Variables**/

/********Input Capture Variables*************/
volatile float current_speed = 0;

float speed_avg[10];
float sum_speed;
float avg_speed;

volatile uint8_t gu8_State = IC_IDLE;
volatile uint8_t no_tooth = 0;
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile float gu32_Freq = 0;
volatile float speed_ic = 0;
volatile uint32_t gu16_TIM2_OVC = 0;
/********Averaging IC Variables*************/
volatile uint32_t tick_arr[25];
volatile uint32_t sum_tick = 0;
volatile uint32_t avg_tick = 0;
volatile uint8_t cnt = 0;
volatile uint8_t calc_complete = 0;
/********Averaging IC Variables*************/
volatile float pre_error;
volatile int integral;
volatile float error;
volatile float derivative;
/********Input Capture Variables*************/

/*Load cell Variables*/
HX711 *OIL_LOADCELL;
HX711 *FLOUR_LOADCELL;
HX711 *WATER_LOADCELL;

volatile uint16_t sum = 0;
uint8_t data[4];
/*Load cell Variables*/

#if	FLOUR_PRESENCE_SENSOR
/*Flour Connector Presence Variables*/
const uint8_t flourConnectorTOFAddress = 0xA4;
float flourConnectorDistanceinMM = 0;
/*Flour Connector Presence Variables*/
#endif


/*Process Structures*/

/*Loadcell Values structure*/
loadcellStruct *flourLoadCellValue, *waterLoadCellValue, *oilLoadCellValue;
/*Loadcell Values structure*/

/*Ejector & leadscrew motor variables*/
stepperLeadscrew kneadLeadscrewTravel, ejectorLeadscrewTravel;
/*Ejector & leadscrew motor variables*/

/*Proximity Sensor Variables*/
proxCnt proximityCnt;
proximity pnpProximityValues;
/*Proximity Sensor Variables*/

/*Press Sensor Variables*/
LPS_t LPS_struct;
temperature_t kTypeTemperature;
/*Press Sensor Variables*/

/*Process Timer structures*/
countVal  timerCount;

/*Process Timer structures*/

/*Stepper Motor Structures*/
Stepper_Motor Ejecter_Stepper_Motor, Knead_screw_Stepper_Motor, Flour_Stepper_Motor, Oil_Stepper_Motor, Water_Stepper_Motor;
/*Stepper Motor Structures*/

/*Error State variables*/
errCnt_t errorCntVal;
errRetry_t errorRetryVal;
errorState_t processErrorState;
/*Error State variables*/

/*FeedRate*/
feedrate_t flourFeedrate;
/*FeedRate*/

/*Process Structures*/

/*Dispensing Variables*/
volatile float calibrationFlourRotPerGmFromAndroid, calibrationWaterRotPerGmFromAndroid,calibrationOilRotPerGmFromAndroid;
uint32_t water_pulse_count;
volatile uint32_t water_value = 0;
uint32_t flourRunTimeFromAndroid = 0, waterRunTimeFromAndroid = 0, oilRunTimeFromAndroid = 0;
/*Dispensing Variables*/



/*Press Variables*/
float pressStopArr[250] = {0};
uint8_t pressInc = 0;
volatile uint32_t lpsMinimumtime = 0;
volatile uint32_t pressReverseForceTime = 0, pressTopPositionErrorCnt = 0;
volatile uint32_t bakingValue,bakingValue_left_counter;
volatile uint32_t pressForcePWM, initialPressPWM = 20,reversePressPWM = 20;
uint8_t bakingValue_flag;
uint32_t timeleft_count;
volatile uint32_t reverseDutyCycle = 0, press_acl_cnt = 0, downDutyCycle = 0 , reverseWaitTime = 0;
volatile float thicknessValueFromAndroid = 0.0f;
volatile float thicknessReferenceValueAndroid = 0.0f;
/*Press Variables*/

/*Toggling Variables*/
uint32_t Clockwise_on_time_android ,Toggle_off_time_android;
uint16_t toggle_count = 0;
uint32_t Clockwise_on_time = 0, Anti_Clockwise_on_time = 0,Toggle_off_time = 0, toggle_motor_clockwise_counts = 0, toggle_motor_anticlockwise_count = 0;
/*Clockwise-AntiClockwise*/
uint16_t toggleMotorClockAntiClockCnt = 0;
//uint32_t  toggleClockAnt
/*Clockwise-AntiClockwise*/
uint16_t toggleCnt = 0, togglePWMVal = 0;
uint8_t togglePWMDutyCycle = 0.0f;
float previousTogglePWM = 0, currentTogglePWM = 0;
/*Toggling Variables*/

/*MAX31855 variables*/
uint8_t errorScreenFlag,tempScreenFlag;
uint16_t pTemp;
/*MAX31855 variables*/
extern const int dataval;

/*State update variables for android process update*/
volatile uint8_t kneading_state_android = 0, press_state_android = 0, kneading_process_percent = 0, press_process_percent = 0, dough_ejection_complete = 0, pizza_quantity = 0;
/*State update variables for android process update*/


/*Debug Variables for Testing*/
uint16_t delay_counter;
uint8_t retVal = 0;
//uint32_t flashdestination = PRODUCTION_REPORT_ADDRESS;
uint8_t report_data[100];
uint8_t _counter;
uint16_t SSR_counter=0;
uint32_t led_toggle = 0;
volatile uint32_t usb_cnt = 0;
volatile uint8_t id = 0;

uint32_t time_measure;
uint32_t time_measure_debug_cnt;
uint8_t ref_msg;

uint8_t	kneadingStart = 0;
uint32_t timeCntCheck;

uint8_t pinVal ;
/*Debug Variables for Testing*/

/***********Variable Declarations***********/

/*Flash and Multiple ADC related variables and Macros*/
#define ADC1_CONVERTED_DATA_BUFFER_SIZE ((uint32_t)  8)
uint32_t ADC1_DATA[ADC1_CONVERTED_DATA_BUFFER_SIZE] ;
#define ADC2_CONVERTED_DATA_BUFFER_SIZE ((uint32_t)  2)
uint32_t ADC2_DATA[ADC2_CONVERTED_DATA_BUFFER_SIZE] ;
//uint32_t ADC1_DATA[ADC1_CONVERTED_DATA_BUFFER_SIZE] __attribute__((section(".ARM.__at_0x24000000")));
/*Flash and Multiple ADC related variables and Macros*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void *argument);
void StartMainProcess(void *argument);
void LOADCELL_PROCESS(void *argument);
void TEMPERATURE_CONTROL(void *argument);
void Android_interface(void *argument);
void StartSpeedControl_Process_Task(void *argument);
void Press_Task(void *argument);
void Dispensing_Process_task(void *argument);
void Kneading_Motor_task(void *argument);
void Adc_task(void *argument);
void Cleaning(void *argument);
void Service_Menu(void *argument);
void EjectorTask(void *argument);
void TogglingTask(void *argument);
void Kneading_Flow_Task(void *argument);
void Callback01(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* @brief:  Function to assign the parameters for the process
 * @params: NOTHING
 * @return: NOTHING*/
void parameterValueAssignment(void)
{
	//	leadScrewMotor.stepPerMMRotation = 5;

	//Commented for testing without android on 19-12-2020
#if	androidParameter
	Pizza_setting.FlourType=Pizza_setting.FlourType;
	Pizza_setting.Dough_shape=Pizza_setting.Dough_shape;
	Pizza_setting.Dough_size=Pizza_setting.Dough_size;
	Pizza_setting.Dough_thickness=Pizza_setting.Dough_thickness;
	Pizza_setting.quantity=Pizza_setting.quantity;
	Pizza_setting.baking_time=Pizza_setting.baking_time;//*1000;
	bakingValue = Pizza_setting.baking_time*1000;
	//	bakingValue = 1000;
	//Kneading Process Parameters upto dispensing
	Ingredient_mixing_parameters.Water_Qty_1 = Ingredient_mixing_parameters.Water_Qty_1;
	Ingredient_mixing_parameters.Water_Qty_2 = Ingredient_mixing_parameters.Water_Qty_2;
	Ingredient_mixing_parameters.Water_Qty_3 = Ingredient_mixing_parameters.Water_Qty_3;
	Ingredient_mixing_parameters.Water_Qty_4 = Ingredient_mixing_parameters.Water_Qty_4;
	Ingredient_mixing_parameters.Water_Qty_5 = Ingredient_mixing_parameters.Water_Qty_5;
	Ingredient_mixing_parameters.Flour_Qty1 = Ingredient_mixing_parameters.Flour_Qty1;
	Ingredient_mixing_parameters.Flour_Qty2 = Ingredient_mixing_parameters.Flour_Qty2;
	Ingredient_mixing_parameters.Flour_Qty3 = Ingredient_mixing_parameters.Flour_Qty3;
	Ingredient_mixing_parameters.Oil_Qty1 = Ingredient_mixing_parameters.Oil_Qty1 / 10;		//Receive value as 30 for 3.0 ml
	Ingredient_mixing_parameters.Oil_Qty2 = Ingredient_mixing_parameters.Oil_Qty2 / 10;		//Receive value as 30 for 3.0 ml
	Ingredient_mixing_parameters.Oil_Qty3 = Ingredient_mixing_parameters.Oil_Qty3;		//Receive value as 3 for 3 ml;
	Ingredient_mixing_parameters.Mixing_speed_1 = Ingredient_mixing_parameters.Mixing_speed_1;		//60% PWM
	Ingredient_mixing_parameters.Mixing_time_1 = Ingredient_mixing_parameters.Mixing_time_1 * 1000;
	Ingredient_mixing_parameters.Mixing_time_2 = Ingredient_mixing_parameters.Mixing_time_2 * 1000;
	Ingredient_mixing_parameters.Mixing_time_3 = Ingredient_mixing_parameters.Mixing_time_3 * 1000;
	Ingredient_mixing_parameters.Mixing_time_4 = Ingredient_mixing_parameters.Mixing_time_4 * 1000 ;
	Ingredient_mixing_parameters.Mixing_time_5 = Ingredient_mixing_parameters.Mixing_time_5 * 1000;
	Ingredient_mixing_parameters.Dough_Base_Step_1 = Ingredient_mixing_parameters.Dough_Base_Step_1 / 10; //Added on 11-10-2020
	Ingredient_mixing_parameters.Dough_Base_Step_2 = Ingredient_mixing_parameters.Dough_Base_Step_2 / 10;
	Ingredient_mixing_parameters.Dough_Base_Step_3 = Ingredient_mixing_parameters.Dough_Base_Step_3 / 10;
//	leadScrewDown4 = 7;			//Added on 30-10-2020
	Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_1 = Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_1 * 1000;
	Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_2 =Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_2 * 1000;
	Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_3 = Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_3 * 1000;
	//Kneading Process Parameters upto dispensing


	Kneading_parameters.Kneading_Speed_1=Kneading_parameters.Kneading_Speed_1;		//90% PWM
	Kneading_parameters.Kneading_Speed_2 = Kneading_parameters.Kneading_Speed_2;
	Kneading_parameters.Kneading_Speed_3 = Kneading_parameters.Kneading_Speed_3;
	Kneading_parameters.Kneading_Speed_4 = Kneading_parameters.Kneading_Speed_4;
	Kneading_parameters.Kneading_Speed_5 = Kneading_parameters.Kneading_Speed_5;
	Kneading_parameters.Kneading_Speed_6 = Kneading_parameters.Kneading_Speed_6;
	Kneading_parameters.Kneading_Time_1 = Kneading_parameters.Kneading_Time_1 * 1000;
	Kneading_parameters.Kneading_Time_2 = Kneading_parameters.Kneading_Time_2 * 1000;
	Kneading_parameters.Kneading_Time_3 = Kneading_parameters.Kneading_Time_3 * 1000;
	Kneading_parameters.Kneading_Time_4 = Kneading_parameters.Kneading_Time_4 * 1000;
	Kneading_parameters.Kneading_Time_5 = Kneading_parameters.Kneading_Time_5 * 1000;
	Kneading_parameters.Kneading_Time_6 = Kneading_parameters.Kneading_Time_6 * 1000;
	Kneading_parameters.Dough_Base_Step_5 = Kneading_parameters.Dough_Base_Step_5 / 10;		//2nd
	Kneading_parameters.Dough_Base_Step_4 = Kneading_parameters.Dough_Base_Step_4 / 10;		//1st
	Kneading_parameters.Dough_Base_Step_7 = Kneading_parameters.Dough_Base_Step_7 / 10;		//4th
	Kneading_parameters.Dough_Base_Step_6 = Kneading_parameters.Dough_Base_Step_6 / 10;		//3rd
	Kneading_parameters.Dough_Base_Step_9 = Kneading_parameters.Dough_Base_Step_9 / 10;		//6th
	Kneading_parameters.Dough_Base_Step_8 = Kneading_parameters.Dough_Base_Step_8 / 10;		//5th	//Rounding cycle distance
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_4 = Kneading_parameters.Rounding_Motor_Clock_Wise_time_4 * 1000;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_5 = Kneading_parameters.Rounding_Motor_Clock_Wise_time_5 * 1000;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_6 = Kneading_parameters.Rounding_Motor_Clock_Wise_time_6 * 1000;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_7 = Kneading_parameters.Rounding_Motor_Clock_Wise_time_7 * 1000;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_8 = Kneading_parameters.Rounding_Motor_Clock_Wise_time_8 * 1000;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_9 = Kneading_parameters.Rounding_Motor_Clock_Wise_time_9 * 1000;


	Coating_Rounding.Dough_Base_Step_10 = Coating_Rounding.Dough_Base_Step_10 / 10;		//Not Used
	Coating_Rounding.Coating_Speed = Coating_Rounding.Coating_Speed; 		//90% PWM
	Coating_Rounding.Flour_Coating_qty = Coating_Rounding.Flour_Coating_qty;
	Coating_Rounding.oil_coating_qty = Coating_Rounding.oil_coating_qty / 10;
	Coating_Rounding.water_coating_qty = Coating_Rounding.water_coating_qty;

	Coating_Rounding.pre_coating_clockwise_anticlockwise_count = Coating_Rounding.pre_coating_clockwise_anticlockwise_count;
	Coating_Rounding.pre_coating_clockwise_time = (Coating_Rounding.pre_coating_clockwise_time * 100);
	Coating_Rounding.pre_coating_anticlockwise_time = (Coating_Rounding.pre_coating_anticlockwise_time * 100);
	Coating_Rounding.pre_coating_delay_between_shifts = (Coating_Rounding.pre_coating_delay_between_shifts * 100);

	Coating_Rounding.Rouding_Clock_wise_cycles = Coating_Rounding.Rouding_Clock_wise_cycles;
	Coating_Rounding.Rounding_time = Coating_Rounding.Rounding_time * 1000;
	Coating_Rounding.delay_between_cycles = Coating_Rounding.delay_between_cycles  * 100;
	Coating_Rounding.Roudnig_motor_clockwise_and_anitclockwise_cycles = Coating_Rounding.Roudnig_motor_clockwise_and_anitclockwise_cycles;
	Coating_Rounding.clockwise_time = Coating_Rounding.clockwise_time * 100;
	Coating_Rounding.Anti_clock_wise_time = Coating_Rounding.Anti_clock_wise_time * 100;
	Coating_Rounding.delay_between_directions_shift = Coating_Rounding.delay_between_directions_shift * 100;


	Dough_Pizza_Eject.Dough_Base_Step_11 = Dough_Pizza_Eject.Dough_Base_Step_11 / 10;
	Dough_Pizza_Eject.Dough_release_cycle_clockwise_and_anticlockwise = Dough_Pizza_Eject.Dough_release_cycle_clockwise_and_anticlockwise;
	Dough_Pizza_Eject.Dough_release_clockwise_time = Dough_Pizza_Eject.Dough_release_clockwise_time * 100;
	Dough_Pizza_Eject.Dough_releae_anit_clokcwise_time = Dough_Pizza_Eject.Dough_releae_anit_clokcwise_time * 100;
	Dough_Pizza_Eject.Delay_between_cycles = Dough_Pizza_Eject.Delay_between_cycles * 100;

	Dough_Pizza_Eject.Ejector_forward_speed1 = Dough_Pizza_Eject.Ejector_forward_speed1;
	Dough_Pizza_Eject.Ejector_forward_speed_2 = Dough_Pizza_Eject.Ejector_forward_speed_2;
	Dough_Pizza_Eject.Ejector_Forward_speed_3 = Dough_Pizza_Eject.Ejector_Forward_speed_3;
	Dough_Pizza_Eject.Ejector_Reverse_speed = Dough_Pizza_Eject.Ejector_Reverse_speed;
	Dough_Pizza_Eject.Ejector_Forward_Distance_1 = Dough_Pizza_Eject.Ejector_Forward_Distance_1;
	Dough_Pizza_Eject.Ejector_Centre_Distance = Dough_Pizza_Eject.Ejector_Centre_Distance;
	/*Added for protection purpose with BTN7960 driver*/
#if	!CYTRON_DRV
	if(Pressing_Baking.initial_Press_Motor_Speed >= 50)
	{
		Pressing_Baking.initial_Press_Motor_Speed = 50;
	}
	else if(Pressing_Baking.initial_Press_Motor_Speed < 50)
	{
		Pressing_Baking.initial_Press_Motor_Speed = Pressing_Baking.initial_Press_Motor_Speed;
	}
	Pressing_Baking.Intial_Press_Motor_running_time = Pressing_Baking.Intial_Press_Motor_running_time * 1000;
	if(Pressing_Baking.Pressing_Force_Duty_cycle >= 35)
	{
		Pressing_Baking.Pressing_Force_Duty_cycle = 35;
	}
	else if(Pressing_Baking.Pressing_Force_Duty_cycle < 35)
	{
		Pressing_Baking.Pressing_Force_Duty_cycle = Pressing_Baking.Pressing_Force_Duty_cycle;
	}
	/*Added for protection purpose with BTN7960 driver*/
//	Pressing_Baking.Thickness_Of_pizza = Pressing_Baking.Thickness_Of_pizza/10;
	thicknessValueFromAndroid = (float)Pressing_Baking.Thickness_Of_pizza/10;
	Pressing_Baking.Holding_Time = Pressing_Baking.Holding_Time * 100;
	if(Pressing_Baking.Press_Motor_reverse_duty_cycle >= 60)
	{
		Pressing_Baking.Press_Motor_reverse_duty_cycle = 60;
	}
	else if(Pressing_Baking.Press_Motor_reverse_duty_cycle < 60)
	{
		Pressing_Baking.Press_Motor_reverse_duty_cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
	}
#elif CYTRON_DRV
	Pressing_Baking.initial_Press_Motor_Speed = Pressing_Baking.initial_Press_Motor_Speed;
	Pressing_Baking.Intial_Press_Motor_running_time = Pressing_Baking.Intial_Press_Motor_running_time * 1000;
	Pressing_Baking.Pressing_Force_Duty_cycle = Pressing_Baking.Pressing_Force_Duty_cycle;
	thicknessValueFromAndroid = (float)Pressing_Baking.Thickness_Of_pizza/10;
	Pressing_Baking.Holding_Time = Pressing_Baking.Holding_Time * 100;
	Pressing_Baking.Press_Motor_reverse_duty_cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
#endif
	Pressing_Baking.Top_Pan_Cut_off_Temp = Pressing_Baking.Top_Pan_Cut_off_Temp;
	Pressing_Baking.Bottom_Pan_Cut_off_Temp = Pressing_Baking.Bottom_Pan_Cut_off_Temp;
	if(Pressing_Baking.Reverse_Force_Duty_Cycle >= 60)
	{
		Pressing_Baking.Reverse_Force_Duty_Cycle = 60;
	}
	else if(Pressing_Baking.Reverse_Force_Duty_Cycle < 60 && Pressing_Baking.Reverse_Force_Duty_Cycle > 0)
	{
		Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Reverse_Force_Duty_Cycle;		//Updated with Android on 17-03
	}
	if(Pressing_Baking.Reverse_Speed_Running_Time > 0 )
	{
		Pressing_Baking.Reverse_Speed_Running_Time = (Pressing_Baking.Reverse_Speed_Running_Time * 1000);	//Updated with Android on 17-03
	}
	else
	{
		Pressing_Baking.Reverse_Speed_Running_Time = PRESS_REVERSE_RUNNING_TIME_MS;
	}



	/*Added on 04-01-2021*/
	if(commonMCSetting.Gap_between_blade_and_dough_base > 0)
	{
		commonMCSetting.Gap_between_blade_and_dough_base = commonMCSetting.Gap_between_blade_and_dough_base/10;
	}
	else if(commonMCSetting.Gap_between_blade_and_dough_base <= 0)
	{
		commonMCSetting.Gap_between_blade_and_dough_base = 1.0;
	}
	//Updated with Android on 17-03
	if(commonMCSetting.Kneader_Base_Cleaning_Position > 0)
	{
		commonMCSetting.Kneader_Base_Cleaning_Position = commonMCSetting.Kneader_Base_Cleaning_Position;
	}
	else if(commonMCSetting.Kneader_Base_Cleaning_Position <= 0)
	{
		commonMCSetting.Kneader_Base_Cleaning_Position = 150;
	}
	if(commonMCSetting.Water_cleaning_time > 0)
	{
		commonMCSetting.Water_cleaning_time = commonMCSetting.Water_cleaning_time * 1000;
	}
	if(commonMCSetting.Water_priming_time > 0)
	{
		commonMCSetting.Water_priming_time = commonMCSetting.Water_priming_time * 1000;
	}
	if(commonMCSetting.Oil_cleaning_time > 0)
	{
		commonMCSetting.Oil_cleaning_time = commonMCSetting.Oil_cleaning_time * 1000;
	}
	if(commonMCSetting.Oil_priming_time > 0)
	{
		commonMCSetting.Oil_priming_time = commonMCSetting.Oil_priming_time * 1000;
	}
	if(commonMCSetting.Flour_priming_time > 0)
	{
		commonMCSetting.Flour_priming_time = commonMCSetting.Flour_priming_time * 1000;
	}
	//Updated with Android on 17-03
	//Added on 24-03-21
	if(commonMCSetting.Idle_top_pan_temperature > 0)
	{
		commonMCSetting.Idle_top_pan_temperature = commonMCSetting.Idle_top_pan_temperature;
	}
	if(commonMCSetting.Idle_top_pan_temperature <= 0)
	{
		commonMCSetting.Idle_top_pan_temperature = 110;
	}
	if(commonMCSetting.Idle_bottom_pan_temperatuer > 0)
	{
		commonMCSetting.Idle_bottom_pan_temperatuer = commonMCSetting.Idle_bottom_pan_temperatuer;
	}
	if(commonMCSetting.Idle_top_pan_temperature <= 0)
	{
		commonMCSetting.Idle_bottom_pan_temperatuer = 120;
	}
	//Added on 24-03-21

	/*Added on 04-01-2021*/

	/*Hardware & Machine Settings 13-01-2021*/

	/*Water Pump Setting*/
	if(HW_Setting1.Water_Pump_Speed > 0)
	{
		Water_Stepper_Motor.rpm = HW_Setting1.Water_Pump_Speed;
	}
	else if(HW_Setting1.Water_Pump_Speed == 0)
	{
		Water_Stepper_Motor.rpm = WATER_STEPPER_DEFAULT_RPM;
	}
	//Modfied on 24-03
	if(HW_Setting1.Water_Pump_Flow_Rate > 0)
	{
		HW_Setting1.Water_Pump_Flow_Rate = ((STEPS_PER_ROTATION_HALF_STEP * 100) / (HW_Setting1.Water_Pump_Flow_Rate));
	}
	//Modfied on 24-03
	else
	{
		HW_Setting1.Water_Pump_Flow_Rate = WATER_DISPENSE_STEPS_PER_ML;
	}
	/*Water Pump Setting*/
	/*Oil Pump Setting*/
	if(HW_Setting1.Oil_Pump_Speed > 0)
	{
		Oil_Stepper_Motor.rpm = HW_Setting1.Water_Pump_Speed;
	}
	else if(HW_Setting1.Oil_Pump_Speed == 0)
	{
		Oil_Stepper_Motor.rpm = OIL_STEPPER_DEFAULT_RPM;
	}
	//Modfied on 24-03
	if(HW_Setting1.Oil_Pump_Flow_Rate > 0)
	{
		HW_Setting1.Oil_Pump_Flow_Rate = ((STEPS_PER_ROTATION_HALF_STEP * 100) / (HW_Setting1.Oil_Pump_Flow_Rate));
	}
	//Modfied on 24-03
	else
	{
		HW_Setting1.Oil_Pump_Flow_Rate = OIL_DISPENSE_STEPS_PER_ML;
	}
	/*Oil Pump Setting*/
	/*Flour Motor Setting*/
	if(HW_Setting1.Flour_Motor_Speed > 0)
	{
		Flour_Stepper_Motor.rpm = HW_Setting1.Flour_Motor_Speed;
	}
	else if(HW_Setting1.Water_Pump_Speed == 0)
	{
		Flour_Stepper_Motor.rpm = FLOUR_STEPPER_DEFAULT_RPM;
	}
	//Modfied on 24-03
	if(HW_Setting1.Flour_Flow_rate_at_level_5 > 0)
	{
		HW_Setting1.Flour_Flow_rate_at_level_5 = ((STEPS_PER_ROTATION_HALF_STEP * 10) / HW_Setting1.Flour_Flow_rate_at_level_5);
	}
	//Modfied on 24-03
	else
	{
		HW_Setting1.Flour_Flow_rate_at_level_5 = FLOUR_DISPENSE_STEPS_PER_GM;
	}
	/*Flour Motor Setting*/


	/*Lead Screw Motor Setting*/
	if(HW_Setting1.Kneading_leadscrew_motor_Speed > 0)
	{
		Knead_screw_Stepper_Motor.rpm = HW_Setting1.Kneading_leadscrew_motor_Speed;
	}
	else if(HW_Setting1.Kneading_leadscrew_motor_Speed== 0)
	{
		Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
	}
	/*Lead Screw Motor Setting*/

	/*Rounding Motor Setting*/
	if(HW_Setting1.Rounding_Motor_Speed > 0)
	{
		HW_Setting1.Rounding_Motor_Speed = HW_Setting1.Rounding_Motor_Speed;
	}
	else
	{
		HW_Setting1.Rounding_Motor_Speed = ROUNDING_DEFAULT_PWM_VALUE;
	}
	/*Added on 18-03-21*/
	if(HW_Setting1.No_Of_pulser_per_ml > 0)
	{
		HW_Setting1.No_Of_pulser_per_ml = (HW_Setting1.No_Of_pulser_per_ml);
	}
	else
	{
		HW_Setting1.No_Of_pulser_per_ml = ((float)1600/ (float)760);			//1600 pulses for 760 ml of water per ml pulse value
	}
	if(HW_Setting1.Oil_loadcell_Min_Level > 0)
	{
		HW_Setting1.Oil_loadcell_Min_Level = HW_Setting1.Oil_loadcell_Min_Level;
	}
	else
	{
		HW_Setting1.Oil_loadcell_Min_Level = OIL_MIN_WT ;
	}
	if(HW_Setting1.Oil_loadcell_Max_Level > 0)
	{
		HW_Setting1.Oil_loadcell_Max_Level = HW_Setting1.Oil_loadcell_Max_Level;
	}
	else
	{
		HW_Setting1.Oil_loadcell_Max_Level = MAX_OIL_WEIGHT ;
	}
	if(HW_Setting1.Water_loadcell_Min_Level > 0)
	{
		HW_Setting1.Water_loadcell_Min_Level = HW_Setting1.Water_loadcell_Min_Level;
	}
	else
	{
		HW_Setting1.Water_loadcell_Min_Level = WATER_MIN_WT ;
	}
	if(HW_Setting1.Water_loadcell_Max_Level > 0)
	{
		HW_Setting1.Water_loadcell_Max_Level = HW_Setting1.Water_loadcell_Max_Level;
	}
	else
	{
		HW_Setting1.Water_loadcell_Max_Level = MAX_WATER_WEIGHT ;
	}
	if(HW_Setting1.Flour_Level_0 > 0)
	{
		HW_Setting1.Flour_Level_0 = HW_Setting1.Flour_Level_0;
	}
	else
	{
		HW_Setting1.Flour_Level_0 = FLOUR_MIN_WT ;
	}
	if(HW_Setting1.Flour_Level_5 > 0)
	{
		HW_Setting1.Flour_Level_5 = HW_Setting1.Flour_Level_5;
	}
	else
	{
		HW_Setting1.Flour_Level_5 = MAX_FLOUR_WEIGHT ;
	}
	/*Press Setting*/
	if(HW_Setting1.Thickness_Reference_value > 0)
	{
		thicknessReferenceValueAndroid = (float)HW_Setting1.Thickness_Reference_value/10.0;
	}
	else
	{
		thicknessReferenceValueAndroid = PIZZA_BASE_THICKNESS_CUTOFF_MM;
	}
	/*Press Setting*/

	/*Loadcell Setting */
    if(Offset_Values_Setting.Flour_Loadcell_Offset_Value > 0)
	{
    	Offset_Values_Setting.Flour_Loadcell_Offset_Value = Offset_Values_Setting.Flour_Loadcell_Offset_Value;
	}
    else
    {
    	Offset_Values_Setting.Flour_Loadcell_Offset_Value = FLOUR_LOADCELL_WEIGHT_OFFSET;
    }
    if(Offset_Values_Setting.Water_Loadcell_Offset_Value > 0)
	{
    	Offset_Values_Setting.Water_Loadcell_Offset_Value = Offset_Values_Setting.Water_Loadcell_Offset_Value;
	}
    else
    {
    	Offset_Values_Setting.Water_Loadcell_Offset_Value = WATER_LOADCELL_WEIGHT_OFFSET;
    }
    if(Offset_Values_Setting.Oil_Loadcell_Offset_Value > 0)
	{
    	Offset_Values_Setting.Oil_Loadcell_Offset_Value = Offset_Values_Setting.Oil_Loadcell_Offset_Value;
	}
    else
    {
    	Offset_Values_Setting.Oil_Loadcell_Offset_Value = OIL_LOADCELL_WEIGHT_OFFSET;
    }
    /*Loadcell Setting */

	/*Added on 18-03-21*/

    /*Added on 24-03 Flour Feedrate*/
    if(HW_Setting1.Feed_Rate_1 > 0)
	{
    	flourFeedrate.feedrate1 = (float)((float)HW_Setting1.Feed_Rate_1 / (float)100);
	}
    else
    {
    	flourFeedrate.feedrate1 = FLOUR_DISPENSE_STG1_FACTOR1;
    }
    if(HW_Setting1.Feed_Rate_2 > 0)
	{
    	flourFeedrate.feedrate2 = (float)((float)HW_Setting1.Feed_Rate_2 /(float) 100);
	}
    else
    {
    	flourFeedrate.feedrate2 = FLOUR_DISPENSE_STG1_FACTOR2;
    }
    if(HW_Setting1.Feed_Rate_3 > 0)
	{
    	flourFeedrate.feedrate3 =(float) ((float)HW_Setting1.Feed_Rate_3 / (float)100);
	}
    else
    {
    	flourFeedrate.feedrate3 = FLOUR_DISPENSE_STG1_FACTOR3;
    }
    if(HW_Setting1.Feed_Rate_4  > 0)
	{
    	flourFeedrate.feedrate4 = (float)( (float)HW_Setting1.Feed_Rate_4 / (float)100);
	}
    else
    {
    	flourFeedrate.feedrate4 = FLOUR_DISPENSE_STG1_FACTOR4;
    }
    if(HW_Setting1.Feed_Rate_5 > 0)
	{
    	flourFeedrate.feedrate5 = ((float)HW_Setting1.Feed_Rate_5 / (float)100);
	}
    else
    {
    	flourFeedrate.feedrate5 = (float)FLOUR_DISPENSE_STG1_FACTOR5;
    }

	Oil_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = HW_Setting1.Oil_Pump_Flow_Rate;
	Water_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = HW_Setting1.Water_Pump_Flow_Rate;
    /*Added on 24-03 Flour Feedrate*/
	/*Hardware & Machine Settings 13-01-2021*/


#elif	!androidParameter
	Pizza_setting.FlourType=0;
	Pizza_setting.Dough_shape=0;
	Pizza_setting.Dough_size=0;
	Pizza_setting.Dough_thickness=3.0;
	Pizza_setting.quantity = 2000;
	Pizza_setting.baking_time=3000;//*1000;
	bakingValue = Pizza_setting.baking_time;
	//*1000;
	/*Kneading Process Parameters upto dispensing*/
	Ingredient_mixing_parameters.Water_Qty_1 = 20;
	Ingredient_mixing_parameters.Water_Qty_2 = 35;//23;
	Ingredient_mixing_parameters.Water_Qty_3 = 32;//37;
	Ingredient_mixing_parameters.Water_Qty_4 = 0;
	Ingredient_mixing_parameters.Water_Qty_5 = 0;
	Ingredient_mixing_parameters.Flour_Qty1 = 65;
	Ingredient_mixing_parameters.Flour_Qty2 = 45;
	Ingredient_mixing_parameters.Flour_Qty3 = 15;
	Ingredient_mixing_parameters.Oil_Qty1 = 3;		//Receive value as 30 for 3.0 ml
	Ingredient_mixing_parameters.Oil_Qty2 = 0;		//Receive value as 30 for 3.0 ml
	Ingredient_mixing_parameters.Oil_Qty3 = 0;		//Receive value as 3 for 3 ml;
	Ingredient_mixing_parameters.Mixing_speed_1 = 280;//Ingredient_mixing_parameters.Mixing_speed_1;		//60% PWM
	Ingredient_mixing_parameters.Mixing_time_1 = 0;
	Ingredient_mixing_parameters.Mixing_time_2 = 0;
	Ingredient_mixing_parameters.Mixing_time_3 = 0;
	Ingredient_mixing_parameters.Mixing_time_4 = 0;
	Ingredient_mixing_parameters.Mixing_time_5 = 10000;
	commonMCSetting.Gap_between_blade_and_dough_base = 1;
	Ingredient_mixing_parameters.Dough_Base_Step_1 = 3; //Added on 11-10-2020
	Ingredient_mixing_parameters.Dough_Base_Step_2 = 6;
	Ingredient_mixing_parameters.Dough_Base_Step_3 = 9;
//	leadScrewDown4 = 7;			//Added on 30-10-2020
	Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_1 = 0;
	Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_2 = 0;
	Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_3 = 1000;
	/*Kneading Process Parameters upto dispensing*/


	Kneading_parameters.Kneading_Speed_1 = 360;//Kneading_parameters.Kneading_Speed_1;		//90% PWM
	Kneading_parameters.Kneading_Speed_2 = 360;//Kneading_parameters.Kneading_Speed_2;
	Kneading_parameters.Kneading_Speed_3 = 360;//Kneading_parameters.Kneading_Speed_3;
	Kneading_parameters.Kneading_Speed_4 = 360;//Kneading_parameters.Kneading_Speed_4;
	Kneading_parameters.Kneading_Speed_5 = 360;//Kneading_parameters.Kneading_Speed_5;
	Kneading_parameters.Kneading_Speed_6 = 360;//Kneading_parameters.Kneading_Speed_6;
	Kneading_parameters.Kneading_Time_1 = 10000;
	Kneading_parameters.Kneading_Time_2 = 10000;
	Kneading_parameters.Kneading_Time_3 = 10000;
	Kneading_parameters.Kneading_Time_4 = 5000;
	Kneading_parameters.Kneading_Time_5 = 5000;
	Kneading_parameters.Kneading_Time_6 = 10000;
	Kneading_parameters.Dough_Base_Step_5 = 10;		//2nd
	Kneading_parameters.Dough_Base_Step_4 = 3;		//1st
	Kneading_parameters.Dough_Base_Step_7 = 15;		//4th
	Kneading_parameters.Dough_Base_Step_6 = 6;		//3rd
	Kneading_parameters.Dough_Base_Step_9 = 15;		//6th
	Kneading_parameters.Dough_Base_Step_8 = 20;		//5th	//Rounding cycle distance
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_4 = 0;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_5 = 0;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_6 = 0;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_7 = 0;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_8 = 0;
	Kneading_parameters.Rounding_Motor_Clock_Wise_time_9 = 0;


	Coating_Rounding.Dough_Base_Step_10 = 80;
	Coating_Rounding.Coating_Speed = 260;//Coating_Rounding.Coating_Speed; 		//90% PWM
	Coating_Rounding.Flour_Coating_qty = 4;
	Coating_Rounding.oil_coating_qty = 0;
	Coating_Rounding.water_coating_qty = 0;

	Coating_Rounding.Rouding_Clock_wise_cycles = 0;
	Coating_Rounding.Rounding_time = 1000;
	Coating_Rounding.delay_between_cycles = 2000;
	Coating_Rounding.Roudnig_motor_clockwise_and_anitclockwise_cycles = 1;
	Coating_Rounding.clockwise_time = 1000;
	Coating_Rounding.Anti_clock_wise_time = 1000;
	Coating_Rounding.delay_between_directions_shift = 500;
	Coating_Rounding.pre_coating_clockwise_anticlockwise_count = 1;
	Coating_Rounding.pre_coating_clockwise_time = 1000;
	Coating_Rounding.pre_coating_anticlockwise_time = 1000;
	Coating_Rounding.pre_coating_delay_between_shifts = 500;


	Dough_Pizza_Eject.Dough_Base_Step_11 = 80;
	Dough_Pizza_Eject.Dough_release_cycle_clockwise_and_anticlockwise = 0;
	Dough_Pizza_Eject.Dough_release_clockwise_time = 100;
	Dough_Pizza_Eject.Dough_releae_anit_clokcwise_time = 100;
	Dough_Pizza_Eject.Delay_between_cycles = 100;

	Dough_Pizza_Eject.Ejector_forward_speed1 = 100;//Dough_Pizza_Eject.Ejector_forward_speed1;
	Dough_Pizza_Eject.Ejector_forward_speed_2 = STEPPER_MOTOR_DEFAULT_RPM;//Dough_Pizza_Eject.Ejector_forward_speed_2;
	Dough_Pizza_Eject.Ejector_Forward_speed_3 = 500;//Dough_Pizza_Eject.Ejector_Forward_speed_3;
	Dough_Pizza_Eject.Ejector_Reverse_speed = 500;//Dough_Pizza_Eject.Ejector_Reverse_speed;
	Dough_Pizza_Eject.Ejector_Forward_Distance_1 = 80;//Dough_Pizza_Eject.Ejector_Forward_Distance_1;
	Dough_Pizza_Eject.Ejector_Centre_Distance = 185;//Dough_Pizza_Eject.Ejector_Centre_Distance;

	Pressing_Baking.initial_Press_Motor_Speed = 100;
	Pressing_Baking.Intial_Press_Motor_running_time = 4 * 1000;
	Pressing_Baking.Pressing_Force_Duty_cycle = 50;
	thicknessValueFromAndroid = 3.0;
	Pressing_Baking.Holding_Time = 500;
	Pressing_Baking.Press_Motor_reverse_duty_cycle = 90;
	Pressing_Baking.Top_Pan_Cut_off_Temp = 110;
	Pressing_Baking.Bottom_Pan_Cut_off_Temp = 130;
	Pressing_Baking.Reverse_Force_Duty_Cycle = 50;		//Updated with Android on 17-03
	Pressing_Baking.Reverse_Speed_Running_Time = PRESS_REVERSE_RUNNING_TIME_MS;


	Water_Stepper_Motor.rpm = WATER_STEPPER_DEFAULT_RPM;
	HW_Setting1.Water_Pump_Flow_Rate = WATER_DISPENSE_STEPS_PER_ML;
	Oil_Stepper_Motor.rpm = OIL_STEPPER_DEFAULT_RPM;
	HW_Setting1.Oil_Pump_Flow_Rate = OIL_DISPENSE_STEPS_PER_ML;
	Flour_Stepper_Motor.rpm = FLOUR_STEPPER_DEFAULT_RPM;
	HW_Setting1.Flour_Flow_rate_at_level_5 = FLOUR_DISPENSE_STEPS_PER_GM;
	HW_Setting1.Kneading_leadscrew_motor_Speed = 300;
	Knead_screw_Stepper_Motor.rpm = 300;
	HW_Setting1.Rounding_Motor_Speed = ROUNDING_DEFAULT_PWM_VALUE;
	HW_Setting1.No_Of_pulser_per_ml = ((float)1600/ (float)760);
	HW_Setting1.Oil_loadcell_Min_Level = OIL_MIN_WT ;
	HW_Setting1.Oil_loadcell_Max_Level = MAX_OIL_WEIGHT ;
	HW_Setting1.Water_loadcell_Min_Level = WATER_MIN_WT ;
	HW_Setting1.Water_loadcell_Max_Level = MAX_WATER_WEIGHT ;
	HW_Setting1.Flour_Level_0 = FLOUR_DISPENSE_LEVEL4 ;
	HW_Setting1.Flour_Level_1 = FLOUR_DISPENSE_LEVEL4;
	HW_Setting1.Flour_Level_2 = FLOUR_DISPENSE_LEVEL4;
	HW_Setting1.Flour_Level_3 = FLOUR_DISPENSE_LEVEL3;
	HW_Setting1.Flour_Level_4 = FLOUR_DISPENSE_LEVEL2;
	HW_Setting1.Flour_Level_5 = FLOUR_DISPENSE_LEVEL1 ;
	thicknessReferenceValueAndroid = PIZZA_BASE_THICKNESS_CUTOFF_MM;
	Offset_Values_Setting.Flour_Loadcell_Offset_Value = FLOUR_LOADCELL_WEIGHT_OFFSET;

	Offset_Values_Setting.Water_Loadcell_Offset_Value = WATER_LOADCELL_WEIGHT_OFFSET;

	Offset_Values_Setting.Oil_Loadcell_Offset_Value = OIL_LOADCELL_WEIGHT_OFFSET;

	flourFeedrate.feedrate1 = FLOUR_DISPENSE_STG1_FACTOR1;
	flourFeedrate.feedrate2 = FLOUR_DISPENSE_STG1_FACTOR2;
	flourFeedrate.feedrate3 = FLOUR_DISPENSE_STG1_FACTOR3;
	flourFeedrate.feedrate4 = FLOUR_DISPENSE_STG1_FACTOR4;
	flourFeedrate.feedrate5 = FLOUR_DISPENSE_STG1_FACTOR5;

	Oil_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = HW_Setting1.Oil_Pump_Flow_Rate;
	Water_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = HW_Setting1.Water_Pump_Flow_Rate;

	HW_Setting1.Thickness_Reference_value = PIZZA_BASE_THICKNESS_CUTOFF_MM;

	/*Added on 04-01-2021*/



	//	commonMCSetting.Gap_between_blade_and_dough_base = 1.0;
	//	CommonMachineSettings.roundingMotorSpeedinPWM = 80.0;
	/*Added on 04-01-2021*/
#endif
}
#if 0
/* @brief:  Function to save the data to flash
 * @params: NOTHING
 * @return: NOTHING*/
void save_to_flash(uint32_t *data,uint32_t data_length)
{
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Allow Access to option bytes sector */
	HAL_FLASH_OB_Unlock();

	/* Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Banks = FLASH_BANK_2;//0x08000000;
	EraseInitStruct.Sector=FLASH_SECTOR_7;
	EraseInitStruct.NbSectors=1;
	EraseInitStruct.VoltageRange=FLASH_VOLTAGE_RANGE_3;
	uint32_t PageError;

	volatile uint32_t write_cnt=0, index=0;

	volatile HAL_StatusTypeDef status;
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	while(index < data_length)
	{
		if (status == HAL_OK)
		{
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FLASH_STORAGE+write_cnt, (uint32_t )data);
			if(status == HAL_OK)
			{
				write_cnt += 32;
				index+=32;
				data+=8;
			}
		}
	}
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}
#endif

/* @brief:  Function to initialize load cell pinout and allocate memory to load cell structures
 * @params: NOTHING
 * @return: NOTHING*/
void loadCellInit(void)
{
	OIL_LOADCELL = (HX711 *) malloc(sizeof(HX711));
	WATER_LOADCELL = (HX711 *) malloc(sizeof(HX711));
	FLOUR_LOADCELL = (HX711 *) malloc(sizeof(HX711));

	flourLoadCellValue = (loadcellStruct *) malloc(sizeof(loadcellStruct));
	waterLoadCellValue = (loadcellStruct *) malloc(sizeof(loadcellStruct));
	oilLoadCellValue = (loadcellStruct *) malloc(sizeof(loadcellStruct));

	flourLoadCellValue->actualLoadCellValue = 0;
	flourLoadCellValue->loadCellGainSetValue = 0;
	flourLoadCellValue->loadCellMaskValue=0;
	flourLoadCellValue->loadCellValueAfterTare = 0;

	waterLoadCellValue->actualLoadCellValue = 0;
	waterLoadCellValue->loadCellGainSetValue = 0;
	waterLoadCellValue->loadCellMaskValue=0;
	waterLoadCellValue->loadCellValueAfterTare = 0;

	oilLoadCellValue->actualLoadCellValue = 0;
	oilLoadCellValue->loadCellGainSetValue = 0;
	oilLoadCellValue->loadCellMaskValue=0;
	oilLoadCellValue->loadCellValueAfterTare = 0;

	OIL_LOADCELL->PD_SCK_PinType=GPIOG;
	OIL_LOADCELL->PD_SCK_PinNumber=GPIO_PIN_3;
	OIL_LOADCELL->DOUT_PinType=GPIOG;
	OIL_LOADCELL->DOUT_PinNumber=GPIO_PIN_2;
	OIL_LOADCELL->gain=1;
	OIL_LOADCELL->SCALE = 434;//217

	WATER_LOADCELL->PD_SCK_PinType=GPIOG;
	WATER_LOADCELL->PD_SCK_PinNumber=GPIO_PIN_5;
	WATER_LOADCELL->DOUT_PinType=GPIOG;
	WATER_LOADCELL->DOUT_PinNumber=GPIO_PIN_4;
	WATER_LOADCELL->gain=1;
	WATER_LOADCELL->SCALE = 434;//217

	FLOUR_LOADCELL->PD_SCK_PinType=GPIOG;
	FLOUR_LOADCELL->PD_SCK_PinNumber=GPIO_PIN_8;
	FLOUR_LOADCELL->DOUT_PinType=GPIOG;
	FLOUR_LOADCELL->DOUT_PinNumber=GPIO_PIN_6;
	FLOUR_LOADCELL->gain=1;		//128 Gain
	FLOUR_LOADCELL->SCALE =217;//217
}
/* @brief:  Function to initialize pins to low for unexpected turn on of stepper-motor driver
 * @params: NOTHING
 * @return: NOTHING*/
void gpioInitReset(void)
{
	HAL_GPIO_WritePin(KNEAD_MTR_EN_GPIO_Port, KNEAD_MTR_EN_Pin, RESET);
	HAL_GPIO_WritePin(PRESS_MTR_EN_GPIO_Port, PRESS_MTR_EN_Pin, RESET);
	HAL_GPIO_WritePin(TOGGLING_MTR_EN_GPIO_Port, TOGGLING_MTR_EN_Pin, RESET);
	HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
	HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, RESET);
	HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
	HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin, RESET);
	HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port, WATER_DISP_EN_Pin, RESET);
	HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin, RESET);
	HAL_GPIO_WritePin(SSR_TOP_PAN_GPIO_Port, SSR_TOP_PAN_Pin, SET);
	HAL_GPIO_WritePin(SSR_BOTTOM_PAN_GPIO_Port, SSR_BOTTOM_PAN_Pin,SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
	HAL_Delay(30);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
	Toggling_DC_Motor_Stop();
}

/* @brief:  Function to initialize the values for various parameter required to run
 * @params: NOTHING
 * @return: NOTHING*/
void defaultValueAssignmentForMCToRun(void)
{
	  kneadLeadscrewTravel.maximumMMTravel = KNEADER_MAXIMUM_LENGTH_IN_MM;
	  ejectorLeadscrewTravel.maximumMMTravel = EJECTOR_MAXIMUM_LENGTH_IN_MM;

	  Flour_Stepper_Motor.rpm = FLOUR_STEPPER_DEFAULT_RPM;

	  Water_Stepper_Motor.rpm = WATER_STEPPER_DEFAULT_RPM;

	  Oil_Stepper_Motor.rpm = OIL_STEPPER_DEFAULT_RPM;

	  if(HW_Setting1.Kneading_leadscrew_motor_Speed == 0)
	  {
		Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
	  }
	  if(HW_Setting1.Thickness_Reference_value > 0)
	  {
		thicknessReferenceValueAndroid = (float)HW_Setting1.Thickness_Reference_value/10.0;
	  }
	  else
	  {
		thicknessReferenceValueAndroid = (float)PIZZA_BASE_THICKNESS_CUTOFF_MM;
	  }

	  if(Offset_Values_Setting.Flour_Loadcell_Offset_Value == 0)
	  {
		  Offset_Values_Setting.Flour_Loadcell_Offset_Value = FLOUR_LOADCELL_WEIGHT_OFFSET;
	  }
	  if(Offset_Values_Setting.Water_Loadcell_Offset_Value == 0)
	  {
		  Offset_Values_Setting.Water_Loadcell_Offset_Value = WATER_LOADCELL_WEIGHT_OFFSET;
	  }
	  if(Offset_Values_Setting.Oil_Loadcell_Offset_Value == 0)
	  {
		  Offset_Values_Setting.Oil_Loadcell_Offset_Value = OIL_LOADCELL_WEIGHT_OFFSET;
	  }
	  if(HW_Setting1.No_Of_pulser_per_ml == 0)
	  {
		  HW_Setting1.No_Of_pulser_per_ml = 2;
	  }
}
void moduleWiseTesting(void)
{
	switch(moduleTest)
	{
	case toggleMotorClockwiseTest:
		Toggling_DC_Set_PWM(30, CLOCKWISE);
		timerCount.togglingTimerCnt = 0;
		moduleTest = waitToComplete;
		break;
	case toggleMotorAnticlockwiseTest:
		Toggling_DC_Set_PWM(30, ANTICLOCKWISE);
		timerCount.togglingTimerCnt = 0;
		moduleTest = noTest;
		break;
	case pressMotorCycleTest:
		Press_motor_state = Press_motor_init;
		moduleTest = noTest;
		break;
	case ejectorHomingPositionTest:
		ejecter_home_position_state = ejecter_start;
		Ejecter_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
		moduleTest = noTest;
		break;
	case ejectorFrontEndPositionTest:
		ejecter_front_end_limit_position_state = ejecter_start;
		Ejecter_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
		moduleTest = noTest;
		break;
	case waterStepperMotorTest:
		Water_stepper_motor_state = Water_stepper_motor_init;
		Water_Stepper_Motor.direction = ANTICLOCKWISE;
		Water_Stepper_Motor.total_no_of_steps = 90 * HW_Setting1.Water_Pump_Flow_Rate;
		break;
	case oilStepperMotorTest:
		Oil_stepper_motor_state = Oil_stepper_motor_init;
		Oil_Stepper_Motor.direction = ANTICLOCKWISE;
		Oil_Stepper_Motor.total_no_of_steps = 50 * HW_Setting1.Oil_Pump_Flow_Rate;
		break;
	case flourStepperMotorTest:
		Flour_stepper_motor_state = Flour_stepper_motor_init;
		Flour_Stepper_Motor.direction = CLOCKWISE;
		Flour_Stepper_Motor.total_no_of_steps = 4000;
		break;
	case kneadingMotorTest:
		currentBladePWM = 50.0;
		Kneading_motor_state = Kneading_motor_init;
		timerCount.togglingTimerCnt = 0;
		moduleTest = waitToComplete;
		break;
	case waitToComplete:
		 if(timerCount.togglingTimerCnt >= 20000)
		 {
			 Blade_DC_Set_PWM(0, ANTICLOCKWISE);
			 moduleTest = noTest;
			 timerCount.togglingTimerCnt = 0;
		 }
		 break;
	case kneaderLeadscrewBottomPositionTest:
		Knead_screw_Stepper_Motor.rpm=STEPPER_MOTOR_DEFAULT_RPM;
		knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
		break;
	case kneaderLeadscrewTopPositionTest:
		Knead_screw_Stepper_Motor.rpm=STEPPER_MOTOR_DEFAULT_RPM;
		knead_screw_homing_or_top_end_limit_state = knead_screw_start;
		break;
	case noTest:
		break;
	}
}
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	/* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
//	if(hadc->Instance==ADC1)
//	{
//		SCB_InvalidateDCache_by_Addr((uint32_t *) &ADC1_DATA[0], ADC1_CONVERTED_DATA_BUFFER_SIZE);
//	}
//
//}
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	/* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
//
//
//	if(hadc->Instance==ADC1)
//	{
//		SCB_InvalidateDCache_by_Addr((uint32_t *) &ADC1_DATA[ADC1_CONVERTED_DATA_BUFFER_SIZE/2], ADC1_CONVERTED_DATA_BUFFER_SIZE);
//	}
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

 if(dataval > 10)
 {
	 HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, SET);
	 HAL_Delay(10);
	 HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, RESET);
 }

  HAL_TIM_Base_Start_IT(&htim6);


  /*Set GPIOs to reset state and  Initialize PWM channels*/
  gpioInitReset();
  PWM_Channels_Init();
  /*Set GPIOs to reset state and  Initialize PWM channels*/


  /*Set Default Speeds of stepper motor & Thickness Reference Value*/

  defaultValueAssignmentForMCToRun();



  /*Set Default Speeds of stepper motor & Thickness Reference Value*/

#if !androidParameter
	parameterValueAssignment();
#endif
	/*Initialize GPIOs & Scale and other parameters for loadcells*/
	loadCellInit();
	Loadcell_Read_state = Flour_loadcell;
	/*Initialize GPIOs & Scale and other parameters for loadcells*/
	/*Module Test*/
#if !androidParameter
	machine_initialization_state= machine_initialization;
//	moduleTest = pressMotorCycleTest;
#endif


//	commonMCSetting.Flour_priming_time = 2000;
	//pressMotorCycleTest
	//oilStepperMotorTest;
	//ejectorHomingPositionTest;
	//ejectorFrontEndPositionTest;
	//kneaderLeadscrewBottomPositionTest;
	//kneaderLeadscrewTopPositionTest;
	//waterStepperMotorTest;
	//flourStepperMotorTest
	//kneadingMotorTest
	//toggleMotorClockwiseTest
	//toggleMotorAnticlockwiseTest

//	Blade_DC_Set_PWM(35, ANTICLOCKWISE);
	/*Module Test*/

	/*Start Heating of Pan initially when device is turned on*/
	if(Pressing_Baking.Top_Pan_Cut_off_Temp == 0)
		Pressing_Baking.Top_Pan_Cut_off_Temp = 100;//120;
	if(Pressing_Baking.Bottom_Pan_Cut_off_Temp == 0)
		Pressing_Baking.Bottom_Pan_Cut_off_Temp= 110;//135;
	/*Start Heating of Pan initially when device is turned on*/

	//Start the temperature control Timer
	HAL_TIM_Base_Start_IT(&htim8);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex02 */
  myMutex02Handle = osMutexNew(&myMutex02_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* creation of myBinarySem02 */
  myBinarySem02Handle = osSemaphoreNew(1, 1, &myBinarySem02_attributes);

  /* creation of myBinarySem03 */
  myBinarySem03Handle = osSemaphoreNew(1, 1, &myBinarySem03_attributes);

  /* creation of myCountingSem01 */
  myCountingSem01Handle = osSemaphoreNew(100, 100, &myCountingSem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	if(myTimer01Handle!=NULL)
		  	osTimerStart(myTimer01Handle,1000);
		/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (100, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Main_process_ta */
  Main_process_taHandle = osThreadNew(StartMainProcess, NULL, &Main_process_ta_attributes);

  /* creation of Load_cell */
  Load_cellHandle = osThreadNew(LOADCELL_PROCESS, NULL, &Load_cell_attributes);

  /* creation of Temperature_con */
  Temperature_conHandle = osThreadNew(TEMPERATURE_CONTROL, NULL, &Temperature_con_attributes);

  /* creation of Android_handler */
  Android_handlerHandle = osThreadNew(Android_interface, NULL, &Android_handler_attributes);

  /* creation of SpeedControl_Ta */
  SpeedControl_TaHandle = osThreadNew(StartSpeedControl_Process_Task, NULL, &SpeedControl_Ta_attributes);

  /* creation of Press */
  PressHandle = osThreadNew(Press_Task, NULL, &Press_attributes);

  /* creation of Dispensing_Task */
  Dispensing_TaskHandle = osThreadNew(Dispensing_Process_task, NULL, &Dispensing_Task_attributes);

  /* creation of Kneading_Motor */
  Kneading_MotorHandle = osThreadNew(Kneading_Motor_task, NULL, &Kneading_Motor_attributes);

  /* creation of Adc_Process_Tas */
  Adc_Process_TasHandle = osThreadNew(Adc_task, NULL, &Adc_Process_Tas_attributes);

  /* creation of Cleaning_Task */
  Cleaning_TaskHandle = osThreadNew(Cleaning, NULL, &Cleaning_Task_attributes);

  /* creation of Service_menu */
  Service_menuHandle = osThreadNew(Service_Menu, NULL, &Service_menu_attributes);

  /* creation of Ejector_Task */
  Ejector_TaskHandle = osThreadNew(EjectorTask, NULL, &Ejector_Task_attributes);

  /* creation of Toggling_Task */
  Toggling_TaskHandle = osThreadNew(TogglingTask, NULL, &Toggling_Task_attributes);

  /* creation of kneading_Proces */
  kneading_ProcesHandle = osThreadNew(Kneading_Flow_Task, NULL, &kneading_Proces_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	Android_Process_TaskHandle = osThreadNew(StartAndroidProcessingTask, NULL, &Android_Process_Tas_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 1;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B_OPT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x007074AF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 30-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 30-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 60-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 600-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 600-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 30-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 100-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 30-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 100-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 60-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MAX_SPI4_CS1_Pin|MAX_SPI4_CS2_Pin|PRESS_MTR_DIR_Pin|PRESS_MTR_EN_Pin
                          |NUCLEO_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUZZER_Pin|SPARE_DC_MTR_ENA_Pin|SPARE_DC_MTR_ENB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DC_SSR_OP1_Pin|SSR_TOP_PAN_Pin|SSR_BOTTOM_PAN_Pin|AGITATOR_MTR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPARE_DRV_MTR_DIR_GPIO_Port, SPARE_DRV_MTR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TOGGLING_MTR_EN_Pin|TOGGLING_MTR_DIR_Pin|SPARE_STEP_MTR_DIR_Pin|SPARE_STEP_MTR_PULSE_Pin
                          |OIL_DISP_EN_Pin|OIL_DISP_DIR_Pin|OIL_DISP_PULSE_Pin|SPARE_UART_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|USB_EN_Pin|KNEAD_MTR_EN_Pin
                          |KNEAD_DIR_Pin|SPARE_SPI3_CS1_Pin|SPARE_SPI3_CS2_Pin|LEADSCREW_EN_Pin
                          |LEADSCREW_DIR_Pin|LEADSCREW_PULSE_Pin|EJECTOR_EN_Pin|EJECTOR_DIR_Pin
                          |EJECTOR_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, OIL_LOADCELL_CLK_Pin|WATER_LOADCELL_CLK_Pin|SPARE_LOADCELL_CLK_Pin|WATER_DISP_EN_Pin
                          |WATER_DISP_DIR_Pin|WATER_DISP_PULSE_Pin|FLOUR_DISP_EN_Pin|FLOUR_DISP_DIR_Pin
                          |FLOUR_DISP_PULSE_Pin|SPARE_STEP_MTR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MAX_SPI4_CS1_Pin MAX_SPI4_CS2_Pin PRESS_MTR_DIR_Pin PRESS_MTR_EN_Pin
                           NUCLEO_LED_Pin */
  GPIO_InitStruct.Pin = MAX_SPI4_CS1_Pin|MAX_SPI4_CS2_Pin|PRESS_MTR_DIR_Pin|PRESS_MTR_EN_Pin
                          |NUCLEO_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin SPARE_DC_MTR_ENA_Pin SPARE_DC_MTR_ENB_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|SPARE_DC_MTR_ENA_Pin|SPARE_DC_MTR_ENB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_SSR_OP1_Pin SSR_TOP_PAN_Pin SSR_BOTTOM_PAN_Pin AGITATOR_MTR_DIR_Pin */
  GPIO_InitStruct.Pin = DC_SSR_OP1_Pin|SSR_TOP_PAN_Pin|SSR_BOTTOM_PAN_Pin|AGITATOR_MTR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SPEED_DIR_Pin AGITATOR_MTR_FAULT_Pin */
  GPIO_InitStruct.Pin = SPEED_DIR_Pin|AGITATOR_MTR_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : FLOW_SENSOR_PIN_Pin */
  GPIO_InitStruct.Pin = FLOW_SENSOR_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FLOW_SENSOR_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EMERGENCY_STOP_Pin */
  GPIO_InitStruct.Pin = EMERGENCY_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EMERGENCY_STOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPARE_DRV_MTR_DIR_Pin */
  GPIO_InitStruct.Pin = SPARE_DRV_MTR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPARE_DRV_MTR_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPARE_DRV_MTR_FAULT_Pin */
  GPIO_InitStruct.Pin = SPARE_DRV_MTR_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SPARE_DRV_MTR_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PRESS_TOP_POS_Pin */
  GPIO_InitStruct.Pin = PRESS_TOP_POS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PRESS_TOP_POS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KNEAD_TOP_POS_Pin KNEAD_BOTTOM_POS_Pin OIL_LOADCELL_DATA_Pin WATER_LOADCELL_DATA_Pin
                           SPARE_LOADCELL_DATA_Pin USB_FAULT_Pin */
  GPIO_InitStruct.Pin = KNEAD_TOP_POS_Pin|KNEAD_BOTTOM_POS_Pin|OIL_LOADCELL_DATA_Pin|WATER_LOADCELL_DATA_Pin
                          |SPARE_LOADCELL_DATA_Pin|USB_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : EJECTOR_START_POS_Pin EJECTOR_MID_POS_Pin EJECTOR_END_POS_Pin */
  GPIO_InitStruct.Pin = EJECTOR_START_POS_Pin|EJECTOR_MID_POS_Pin|EJECTOR_END_POS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPARE_POS1_Pin SPARE_POS2_Pin */
  GPIO_InitStruct.Pin = SPARE_POS1_Pin|SPARE_POS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : TOGGLING_MTR_EN_Pin TOGGLING_MTR_DIR_Pin SPARE_STEP_MTR_DIR_Pin SPARE_STEP_MTR_PULSE_Pin
                           OIL_DISP_EN_Pin OIL_DISP_DIR_Pin SPARE_UART_DE_Pin */
  GPIO_InitStruct.Pin = TOGGLING_MTR_EN_Pin|TOGGLING_MTR_DIR_Pin|SPARE_STEP_MTR_DIR_Pin|SPARE_STEP_MTR_PULSE_Pin
                          |OIL_DISP_EN_Pin|OIL_DISP_DIR_Pin|SPARE_UART_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin USB_EN_Pin KNEAD_MTR_EN_Pin
                           KNEAD_DIR_Pin SPARE_SPI3_CS1_Pin SPARE_SPI3_CS2_Pin LEADSCREW_EN_Pin
                           LEADSCREW_DIR_Pin EJECTOR_EN_Pin EJECTOR_DIR_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|USB_EN_Pin|KNEAD_MTR_EN_Pin
                          |KNEAD_DIR_Pin|SPARE_SPI3_CS1_Pin|SPARE_SPI3_CS2_Pin|LEADSCREW_EN_Pin
                          |LEADSCREW_DIR_Pin|EJECTOR_EN_Pin|EJECTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OIL_LOADCELL_CLK_Pin WATER_LOADCELL_CLK_Pin SPARE_LOADCELL_CLK_Pin WATER_DISP_EN_Pin
                           WATER_DISP_DIR_Pin FLOUR_DISP_EN_Pin FLOUR_DISP_DIR_Pin SPARE_STEP_MTR_EN_Pin */
  GPIO_InitStruct.Pin = OIL_LOADCELL_CLK_Pin|WATER_LOADCELL_CLK_Pin|SPARE_LOADCELL_CLK_Pin|WATER_DISP_EN_Pin
                          |WATER_DISP_DIR_Pin|FLOUR_DISP_EN_Pin|FLOUR_DISP_DIR_Pin|SPARE_STEP_MTR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LEADSCREW_PULSE_Pin EJECTOR_PULSE_Pin */
  GPIO_InitStruct.Pin = LEADSCREW_PULSE_Pin|EJECTOR_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : WATER_DISP_PULSE_Pin FLOUR_DISP_PULSE_Pin */
  GPIO_InitStruct.Pin = WATER_DISP_PULSE_Pin|FLOUR_DISP_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OIL_DISP_PULSE_Pin */
  GPIO_InitStruct.Pin = OIL_DISP_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OIL_DISP_PULSE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM16)			//1us resolution
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrupt source is channel 1
		{
			if(gu8_State == IC_IDLE)
			{
				gu32_T1 = TIM16->CCR1;
//				gu32_T1 = __HAL_TIM_GET_COMPARE(&htim16, TIM_CHANNEL_1);
				gu16_TIM2_OVC = 0;
				gu8_State = IC_DONE;
			}
			else if(gu8_State == IC_DONE)
			{
				gu32_T2 = TIM16->CCR1;
//				gu32_T2 = __HAL_TIM_GET_COMPARE(&htim16, TIM_CHANNEL_1);
				/*if (gu32_T2 > gu32_T1)
				{
					gu32_Ticks = gu32_T2-gu32_T1;   // calculate the difference
				}

				else if (gu32_T2 < gu32_T1)
				{
					gu32_Ticks = ((0xffff-gu32_T1)+gu32_T2) +1;
				}
				else
				{
					Error_Handler();
				}*/
				gu32_Ticks = (gu32_T2 + (gu16_TIM2_OVC * 65536)) - gu32_T1;
				tick_arr[cnt] = gu32_Ticks;
				cnt++;
				if(cnt >= 20)
				{
					cnt = 0;
				}
				gu8_State = IC_IDLE;
			}
		}
		else
		{
			current_speed = 0;
			avg_speed = 0;
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(10000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMainProcess */
/**
 * @brief Function implementing the Main_process_ta thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMainProcess */
void StartMainProcess(void *argument)
{
  /* USER CODE BEGIN StartMainProcess */
	moduleWiseTesting();
	/*To start the process*/

#if androidParameter
#if COMPLETE_PROCESS == 1
	machine_initialization_state= machine_initialization;
#elif COMPLETE_PROCESS == 0
	Main_process_state = Main_Start;
#endif
#endif
	/*To start the process*/
	/* Infinite loop */
	for(;;)
	{
//		if(moduleTest == waitToComplete && timerCount.togglingTimerCnt >= 20000)
//		{
//			Toggling_DC_Set_PWM(0, ANTICLOCKWISE);
//			Blade_DC_Set_PWM(0, ANTICLOCKWISE);
//			moduleTest = noTest;
//			timerCount.togglingTimerCnt = 0;
//		}

		//Machine Initialization State
		switch(machine_initialization_state)
		{
		case machine_initialization_idle:
			break;
		case machine_liquid_priming:
			//Add Priming
			machine_initialization_state = water_priming_reverse_start;
			break;
		case machine_initialization:
			if(pnpProximityValues.pressTopPositionLimit==DETECTED)
			{
				machine_initialization_state=ejecter_or_Knead_screw_init;
			}
			else
			{
				machine_initialization_state=press_motor_homing_init;
			}
			break;
		case press_motor_homing_init:
			Pressing_Baking.Press_Motor_reverse_duty_cycle = 60;
			Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
			Press_motor_state=Press_motor_homing_init;
			machine_initialization_state=press_motor_homing_init_wait;
			break;
		case press_motor_homing_init_wait:
			if(Press_motor_state==Press_motor_completed)
			{
				Read_ADC2_Channel_state = ADC2_INIT;
				Press_motor_state=Press_motor_idle;
				machine_initialization_state=ejecter_or_Knead_screw_init;
			}
			break;
		case ejecter_or_Knead_screw_init:
			if(pnpProximityValues.ejectorStartPositionLimit==DETECTED)
			{
				machine_initialization_state=Knead_screw_bottom_end_init;
			}
			else if(pnpProximityValues.leadscrewBottomPositionLimit==DETECTED)
			{
				machine_initialization_state=ejecter_homing_init;
			}
			else
			{
				machine_initialization_state=Knead_screw_top_end_or_homing_init;
			}
			break;
		case ejecter_homing_init:
			Ejecter_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;//Dough_Pizza_Eject.Ejector_Reverse_speed;
			ejecter_home_position_state=ejecter_start;
			machine_initialization_state=ejecter_homing_wait;
			break;
		case ejecter_homing_wait:
			if((ejecter_home_position_state==ejecter_completed)&&(init_completed))
			{
				ejecter_home_position_state=ejecter_idle;
				machine_initialization_state=machine_initialization_complete;
#if		!androidParameter
				Main_process_state = Main_Init;
#elif	androidParameter
				Main_process_state = Main_Start;
#endif
			}
			else if((ejecter_home_position_state==ejecter_completed)&&(!init_completed))
			{
				ejecter_home_position_state=ejecter_idle;
				machine_initialization_state=Knead_screw_bottom_end_init;
			}
			break;
		case ejecter_front_end_init:
			Ejecter_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;//Dough_Pizza_Eject.Ejector_Forward_speed_3;
			ejecter_front_end_limit_position_state=ejecter_start;
			machine_initialization_state=ejecter_front_end_wait;
			break;
		case ejecter_front_end_wait:
			if(ejecter_front_end_limit_position_state==ejecter_completed)
			{
				init_completed=1;
				machine_initialization_state=ejecter_homing_init;
				ejecter_front_end_limit_position_state=ejecter_idle;
			}
			break;
		case Knead_screw_top_end_or_homing_init:
			Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
			machine_initialization_state=knead_screw_top_end_or_homing_wait;
			knead_screw_homing_or_top_end_limit_state=knead_screw_start;
			break;

		case knead_screw_top_end_or_homing_wait:
			if((knead_screw_homing_or_top_end_limit_state==knead_screw_completed)&&(init_completed))
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_idle;
				machine_initialization_state=machine_initialization_complete;
#if		!androidParameter
				Main_process_state = Main_Init;
#elif	androidParameter
				Main_process_state = Main_Start;
#endif
			}
			else if((knead_screw_homing_or_top_end_limit_state==knead_screw_completed)&&(!init_completed))
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_idle;
				Ejecter_Stepper_Motor.rpm = Dough_Pizza_Eject.Ejector_Reverse_speed;
				ejecter_home_position_state=ejecter_start;
				machine_initialization_state=ejecter_homing_wait;
			}
			break;
		case Knead_screw_bottom_end_init:
			Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
			knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_start;
			machine_initialization_state=knead_screw_bottom_end_wait;
			break;
		case knead_screw_bottom_end_wait:
			if(knead_screw_rear_end_or_bottom_end_limit_state==knead_screw_completed)
			{
				knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_idle;
				machine_initialization_state=ejecter_front_end_init;
				//machine_initialization_state = machine_initialization_complete;
#if		!androidParameter
				Main_process_state = Main_Init;
#elif	androidParameter
				Main_process_state = Main_Start;
#endif
			}
			break;
		case water_priming_reverse_start:
			Water_Stepper_Motor.direction = CLOCKWISE;
			Water_Stepper_Motor.total_no_of_steps = WATER_PRIMING_REVERSE_STEPS;
			Water_stepper_motor_state = Water_stepper_motor_init;
			machine_initialization_state = water_priming_reverse_wait_to_complete;
			break;
		case water_priming_reverse_wait_to_complete:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				Water_stepper_motor_state = Water_stepper_motor_idle;
				machine_initialization_state = oil_priming_reverse_start;
			}
			break;
		case oil_priming_reverse_start:
			Oil_Stepper_Motor.direction = CLOCKWISE;
			Oil_Stepper_Motor.total_no_of_steps = OIL_PRIMING_REVERSE_STEPS;
			Oil_stepper_motor_state = Oil_stepper_motor_init;
			machine_initialization_state = oil_priming_reverse_wait_to_complete;
			break;
		case oil_priming_reverse_wait_to_complete:
			if(Oil_stepper_motor_state == Oil_stepper_motor_completed)
			{
				Oil_stepper_motor_state = Oil_stepper_motor_idle;
				machine_initialization_state = water_priming_forward_start;
			}
			break;
		case water_priming_forward_start:
			Water_Stepper_Motor.direction = ANTICLOCKWISE;
			Water_Stepper_Motor.total_no_of_steps = WATER_PRIMING_FORWARD_STEPS;
			Water_stepper_motor_state = Water_stepper_motor_init;
			machine_initialization_state = water_priming_forward_wait_to_complete;

			break;
		case water_priming_forward_wait_to_complete:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				Water_stepper_motor_state = Water_stepper_motor_idle;
				machine_initialization_state = oil_priming_forward_start;
			}
			break;
		case oil_priming_forward_start:
			Oil_Stepper_Motor.direction = ANTICLOCKWISE;
			Oil_Stepper_Motor.total_no_of_steps = OIL_PRIMING_FORWARD_STEPS;
			Oil_stepper_motor_state = Oil_stepper_motor_init;
			machine_initialization_state = oil_priming_forward_wait_to_complete;
			break;
		case oil_priming_forward_wait_to_complete:
			if(Oil_stepper_motor_state == Oil_stepper_motor_completed)
			{
				Oil_stepper_motor_state = Oil_stepper_motor_idle;
				machine_initialization_state = machine_initialization;
			}
			break;
		case machine_initialization_complete:
			break;
		}

		switch(Main_process_state)
		{
		case Main_Idle:

			break;
		case Main_Start:
			//Wait for temperature to reach the minimum temperature required and wait for start command from android
#if COMPLETE_PROCESS == 1
			if(((start1 == 1) && (Pizza_setting.quantity >= 1) && (kTypeTemperature.topPanTemperature >= (Pressing_Baking.Top_Pan_Cut_off_Temp - TEMPERATURE_CUTOFF_VALUE)) && (kTypeTemperature.bottomPanTemperature >= (Pressing_Baking.Bottom_Pan_Cut_off_Temp - TEMPERATURE_CUTOFF_VALUE))) || (start1 == 1 && Pizza_setting.Dough_shape == 1))
			{
				kneadingStart = 1;
				if((pnpProximityValues.ejectorStartPositionLimit != DETECTED || pnpProximityValues.pressTopPositionLimit != DETECTED) && (machine_initialization_state == machine_initialization_complete))
				{
					machine_initialization_state = machine_initialization;
					Main_process_state = Main_Start;
				}
				else if((pnpProximityValues.ejectorStartPositionLimit == DETECTED && pnpProximityValues.pressTopPositionLimit == DETECTED) && (machine_initialization_state == machine_initialization_complete))
				{
					if(HW_Setting1.Kneading_leadscrew_motor_Speed > 0)
					{
						Knead_screw_Stepper_Motor.rpm = HW_Setting1.Kneading_leadscrew_motor_Speed;
					}
					else if(HW_Setting1.Kneading_leadscrew_motor_Speed== 0)
					{
						Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
					}
					Oil_Stepper_Motor.direction = ANTICLOCKWISE;
					Water_Stepper_Motor.direction = ANTICLOCKWISE;
					Flour_Stepper_Motor.direction = CLOCKWISE;
					start1 = 0;
					Main_process_state = Main_Init;
				}
			}
#elif COMPLETE_PROCESS == 0
			if((start1 == 1) && (Pizza_setting.quantity >= 1))
			{
				kneadingStart = 1;
				if(HW_Setting1.Kneading_leadscrew_motor_Speed > 0)
				{
					Knead_screw_Stepper_Motor.rpm = HW_Setting1.Kneading_leadscrew_motor_Speed;
				}
				else if(HW_Setting1.Kneading_leadscrew_motor_Speed== 0)
				{
					Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
				}
				Oil_Stepper_Motor.direction = ANTICLOCKWISE;
				Water_Stepper_Motor.direction = ANTICLOCKWISE;
				Flour_Stepper_Motor.direction = CLOCKWISE;
				start1 = 0;
//				machine_initialization_state = machine_initialization_complete;
				Main_process_state = Main_Init;
			}
#endif
			break;
		case Main_Init:
			//Initialization Completed?
#if COMPLETE_PROCESS  ==  1
			if(machine_initialization_state == machine_initialization_complete)
			{
#if !androidParameter
				Oil_Stepper_Motor.direction = ANTICLOCKWISE;
				Water_Stepper_Motor.direction = ANTICLOCKWISE;
				Flour_Stepper_Motor.direction = CLOCKWISE;
				parameterValueAssignment();
#endif
				kneadingProcessState = kneading_top_proximity_check_init;
				kneading_process_percent = 0;
				Main_process_state = Main_knead;
			}
#elif COMPLETE_PROCESS == 0
			kneadingProcessState = kneading_top_proximity_check_init;
			Main_process_state = Main_knead;
#endif
			break;
		case Main_knead:
			/*If kneading is completed increment the pizza quantity once the dough base reaches the bottom*/
			if(kneadingProcessState == kneading_complete &&  knead_screw_rear_end_or_bottom_end_limit_state == knead_screw_completed)
			{
				pizza_quantity++;
				kneadingProcessState =kneading_idle;
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				Blade_DC_Motor_Stop();
				press_state_android = 6;
				if(Pizza_setting.Dough_shape == PIZZA_BALL_MODE)			//Pizza Ball mode
				{
#if COMPLETE_PROCESS == 1
					Ejecter_Stepper_Motor.rpm = Dough_Pizza_Eject.Ejector_Forward_speed_3;
					ejecter_front_end_limit_position_state = ejecter_start;
					Main_process_state = Main_Eject;
#elif COMPLETE_PROCESS == 0
					Flour_stepper_motor_state = Flour_stepper_motor_idle;
					Water_stepper_motor_state = Water_stepper_motor_idle;
					Oil_stepper_motor_state = Oil_stepper_motor_idle;
					Toggle_motor_main_state = Toggle_motor_idle;
					knead_screw_homing_or_top_end_limit_state = knead_screw_idle;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
					Knead_movement_to_xx_mm_state = knead_screw_idle;
					ejecter_home_position_state = ejecter_idle;
					timerCount.pressTimerCnt = 0;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
					ejecter_home_position_state = ejecter_completed;
					Main_process_state = Main_Eject_Wait;
#endif
				}
				else if(Pizza_setting.Dough_shape == PIZZA_BASE_MODE)			//Pizza Base Mode
				{
					knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_idle;
					Ejecter_Stepper_Motor.rpm = Dough_Pizza_Eject.Ejector_forward_speed1;	//Speed to start of pan
					ejectorLeadscrewTravel.newMMTravel = Dough_Pizza_Eject.Ejector_Forward_Distance_1;		//Distance to start of Pan
					ejecter_movment_to_xx_mm_state = ejecter_start;
					Main_process_state = Main_Eject_doughball_to_start_of_pan;
				}

			}
			break;
		case Main_Eject_doughball_to_start_of_pan:
			//Once kneading is completed eject the dough ball to start of pan at lesser RPM
			if(ejecter_movment_to_xx_mm_state == ejecter_completed)
			{
				press_state_android = 7;
				Ejecter_Stepper_Motor.rpm = Dough_Pizza_Eject.Ejector_forward_speed_2;			//Speed to center of pan
				ejecter_movment_to_xx_mm_state = ejecter_idle;
				ejectorLeadscrewTravel.newMMTravel = (Dough_Pizza_Eject.Ejector_Centre_Distance);					//Distance to center of pan
				ejecter_movment_to_xx_mm_state = ejecter_start;
				Main_process_state = Main_Eject_doughball_start_to_center;
			}
			break;
		case Main_Eject_doughball_start_to_center:
			//Once ejected the start of pan is completed eject to center of pan at different RPM
			if(ejecter_movment_to_xx_mm_state == ejecter_completed)
			{
				press_state_android = 5;
				ejecter_movment_to_xx_mm_state = ejecter_idle;
				Ejecter_Stepper_Motor.rpm = Dough_Pizza_Eject.Ejector_Reverse_speed;
				ejecter_home_position_state = ejecter_start;
				Main_process_state = Main_Ejector_to_home_position;
			}
			break;
		case Main_Ejector_to_home_position:
			if(ejecter_home_position_state == ejecter_completed)
			{
				kneadingStart = 0;
				press_state_android = 5;
				if(Pizza_setting.quantity > 1 && Pizza_setting.quantity > pizza_quantity)
				{
					//Start the kneading once the pizza is placed in center and the required pizza quantity is less than completed quantity & start the press of the earlier pizza dough
					kneadingProcessState = kneading_top_proximity_check_init;
					Flour_stepper_motor_state = Flour_stepper_motor_idle;
					Water_stepper_motor_state = Water_stepper_motor_idle;
					Oil_stepper_motor_state = Oil_stepper_motor_idle;
					Toggle_motor_main_state = Toggle_motor_idle;
					knead_screw_homing_or_top_end_limit_state = knead_screw_idle;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
					Knead_movement_to_xx_mm_state = knead_screw_idle;
					ejecter_home_position_state = ejecter_idle;
					timerCount.pressTimerCnt = 0;
					Main_process_state = Main_press;
					Press_motor_state = Press_motor_init;
				}
				else
				{
					Press_motor_state = Press_motor_init;
					ejecter_home_position_state  = ejecter_idle;
					Main_process_state = Main_press;
					timerCount.pressTimerCnt = 0;
				}
			}
			break;
		case Main_press:
			if(Press_motor_state == Press_motor_completed)
			{
				//Update the baking time
				press_state_android = 4;
#if 			androidParameter
				bakingValue = Pizza_setting.baking_time*1000;		//Commented on 19-12-2020
#endif
				Press_motor_state = Press_motor_idle;
				timerCount.pressTimerCnt = 0;
				bakingValue_left_counter = bakingValue/1000;
				bakingValue_flag = 1;
				timerCount.bakingTimerCnt=0;
				Main_process_state = Main_Baking_Wait;
			}
			break;
		case Main_Baking_Wait:
			//Wait for baking time to be completed
			if(timerCount.bakingTimerCnt >= bakingValue)
			{
				if(machine_initialization_state==machine_initialization_complete)
				{
					press_process_percent = 95;
				}
				bakingValue_flag = 0;
				timerCount.bakingTimerCnt=0;
				Ejecter_Stepper_Motor.rpm = Dough_Pizza_Eject.Ejector_Forward_speed_3;
				ejecter_front_end_limit_position_state = ejecter_start;
				Main_process_state = Main_Eject;
			}
			break;
		case Main_Eject:
			//Eject the pizza base once pressed
			if(ejecter_front_end_limit_position_state == ejecter_completed)
			{
				if(machine_initialization_state==machine_initialization_complete && Pizza_setting.Dough_shape != PIZZA_BALL_MODE)
				{
					press_process_percent = 100;
				}
				if(Pizza_setting.Dough_shape == PIZZA_BALL_MODE)
				{
					Flour_stepper_motor_state = Flour_stepper_motor_idle;
					Water_stepper_motor_state = Water_stepper_motor_idle;
					Oil_stepper_motor_state = Oil_stepper_motor_idle;
					Toggle_motor_main_state = Toggle_motor_idle;
					knead_screw_homing_or_top_end_limit_state = knead_screw_idle;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
					Knead_movement_to_xx_mm_state = knead_screw_idle;
					ejecter_home_position_state = ejecter_idle;
					timerCount.pressTimerCnt = 0;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
				}
				press_state_android = 5;
				ejecter_front_end_limit_position_state = ejecter_idle;
				Ejecter_Stepper_Motor.rpm = Dough_Pizza_Eject.Ejector_Reverse_speed;
				ejecter_home_position_state = ejecter_start;
				Main_process_state = Main_Eject_Wait;
			}
			break;
		case Main_Eject_Wait:
			if(ejecter_home_position_state == ejecter_completed)
			{
				if(Pizza_setting.Dough_shape == PIZZA_BALL_MODE)
				{
					if(pizza_quantity >= Pizza_setting.quantity)
					{
						//If required pizza and completed pizza quantity is obtained indicate that the entire cycle is completed
						dough_ejection_complete = 0;
						Main_process_state = Main_completed;
					}
					else if(pizza_quantity < Pizza_setting.quantity)
					{
						kneadingProcessState = kneading_top_proximity_check_init;
						//If required pizza quantity is not obtained then wait for kneading of the next cycle to complete
						dough_ejection_complete = 1;
						ejecter_home_position_state = ejecter_idle;
						ejecter_front_end_limit_position_state = ejecter_idle;
						Main_process_state = Main_knead;
					}
				}
				else if(Pizza_setting.Dough_shape == PIZZA_BASE_MODE)
				{

					if(pizza_quantity >= Pizza_setting.quantity)
					{
						//If required pizza and completed pizza quantity is obtained indicate that the entire cycle is completed
						dough_ejection_complete = 0;
						Main_process_state = Main_completed;
					}
					else
					{
						//If required pizza quantity is not obtained then wait for kneading of the next cycle to complete
						dough_ejection_complete = 1;
						Press_motor_state = Press_motor_idle;
						ejecter_home_position_state = ejecter_idle;
						ejecter_front_end_limit_position_state = ejecter_idle;
						Main_process_state = Main_knead;		//Push to check for completion of kneading cycle
					}
				}
			}
			break;
		case Main_completed:
			//Indicate android for completion of cycle and push all states to idle for next cycle starting
			dough_ejection_complete = 0;
			pizza_quantity = 0;
			complete_process_done = 1;		//indicate android that process is complete
			kneading_process_percent=0;
			press_process_percent = 0;
			//machine_initialization_state = machine_initialization_idle;
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
			ejecter_front_end_limit_position_state = ejecter_idle;
			Flour_stepper_motor_state = Flour_stepper_motor_idle;
			Oil_stepper_motor_state = Oil_stepper_motor_idle;
			Water_stepper_motor_state = Water_stepper_motor_idle;
			break;
		}
		if(kneadingStart)
		{
			timeCntCheck++;
		}
		osDelay(1);
	}
  /* USER CODE END StartMainProcess */
}

/* USER CODE BEGIN Header_LOADCELL_PROCESS */
/**
 * @brief Function implementing the Load_cell thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LOADCELL_PROCESS */
void LOADCELL_PROCESS(void *argument)
{
  /* USER CODE BEGIN LOADCELL_PROCESS */
	/* Infinite loop */
	for(;;)
	{
		switch(Loadcell_Read_state)
		{
		case LoadCell_IDLE:
			break;
		case Oil_Loadcell:
			if(HAL_GPIO_ReadPin(OIL_LOADCELL->DOUT_PinType, OIL_LOADCELL->DOUT_PinNumber)==0)
			{
				sum=0;
				oilLoadCellValue->loadCellMaskValue=0;
				data[3]=0;
				data[2]=0;
				data[1]=0;
				data[0]=0;
				data[2] = (Hx711_shiftInMsbFirst(OIL_LOADCELL));
				data[1]= (Hx711_shiftInMsbFirst(OIL_LOADCELL));
				data[0]= (Hx711_shiftInMsbFirst(OIL_LOADCELL));
				data[2]=~data[2];
				data[1]=~data[1];
				data[0]=~data[0];
				oilLoadCellValue->loadCellMaskValue =(
						(uint32_t)(data[2]) << 16
						| (uint32_t)(data[1]) << 8
						| (uint32_t)(data[0]) );
				sum=oilLoadCellValue->loadCellMaskValue/OIL_LOADCELL->SCALE;
				oilLoadCellValue->actualLoadCellValue=(0x0000FFFF & sum);
				sum=0;
				// set the channel and the gain factor for the next reading using the clock pin
				for (oilLoadCellValue->loadCellGainSetValue = 0; oilLoadCellValue->loadCellGainSetValue < OIL_LOADCELL->gain; oilLoadCellValue->loadCellGainSetValue++)
				{
					HAL_GPIO_WritePin(OIL_LOADCELL->PD_SCK_PinType, OIL_LOADCELL->PD_SCK_PinNumber, GPIO_PIN_SET);
					HAL_GPIO_WritePin(OIL_LOADCELL->PD_SCK_PinType, OIL_LOADCELL->PD_SCK_PinNumber,GPIO_PIN_RESET);
				}
				sum=oilLoadCellValue->loadCellMaskValue/OIL_LOADCELL->SCALE;
				oilLoadCellValue->actualLoadCellValue = sum;
				if(sum < Offset_Values_Setting.Oil_Loadcell_Offset_Value)
				{
					oilLoadCellValue->loadCellValueAfterTare = 0;
				}
				else
				{
					oilLoadCellValue->loadCellValueAfterTare = (sum - Offset_Values_Setting.Oil_Loadcell_Offset_Value);
				}
			}
			Loadcell_Read_state = Flour_loadcell;
			break;
		case Flour_loadcell:
			if(HAL_GPIO_ReadPin(FLOUR_LOADCELL->DOUT_PinType, FLOUR_LOADCELL->DOUT_PinNumber)==0)
			{
				data[3]=0;
				data[2]=0;
				data[1]=0;
				data[0]=0;
				sum = 0;
				flourLoadCellValue->loadCellMaskValue=0;
				data[2] = (Hx711_shiftInMsbFirst(FLOUR_LOADCELL));
				data[1]= (Hx711_shiftInMsbFirst(FLOUR_LOADCELL));
				data[0]= (Hx711_shiftInMsbFirst(FLOUR_LOADCELL));
				data[2]=~data[2];
				data[1]=~data[1];
				data[0]=~data[0];
				flourLoadCellValue->loadCellMaskValue = (
						(uint32_t)(data[2]) << 16
						| (uint32_t)(data[1]) << 8
						| (uint32_t)(data[0]) );
				// set the channel and the gain factor for the next reading using the clock pin
				for (flourLoadCellValue->loadCellGainSetValue = 0; flourLoadCellValue->loadCellGainSetValue < FLOUR_LOADCELL->gain; flourLoadCellValue->loadCellGainSetValue++)
				{
					HAL_GPIO_WritePin(FLOUR_LOADCELL->PD_SCK_PinType, FLOUR_LOADCELL->PD_SCK_PinNumber, GPIO_PIN_SET);
					HAL_GPIO_WritePin(FLOUR_LOADCELL->PD_SCK_PinType, FLOUR_LOADCELL->PD_SCK_PinNumber,GPIO_PIN_RESET);
				}
				sum=flourLoadCellValue->loadCellMaskValue/FLOUR_LOADCELL->SCALE;
				flourLoadCellValue->actualLoadCellValue = (0x0000FFFF & sum);
				if(sum < Offset_Values_Setting.Flour_Loadcell_Offset_Value)
				{
					flourLoadCellValue->loadCellValueAfterTare = 0;
				}
				else
				{
					flourLoadCellValue->loadCellValueAfterTare = (sum - Offset_Values_Setting.Flour_Loadcell_Offset_Value);
				}
			}
			Loadcell_Read_state = Water_Loadcell;
			break;
		case Water_Loadcell:
			if(HAL_GPIO_ReadPin(WATER_LOADCELL->DOUT_PinType, WATER_LOADCELL->DOUT_PinNumber)==0)
			{
				sum=0;
				waterLoadCellValue->loadCellMaskValue=0;
				data[3]=0;
				data[2]=0;
				data[1]=0;
				data[0]=0;
				data[2] = (Hx711_shiftInMsbFirst(WATER_LOADCELL));
				data[1]= (Hx711_shiftInMsbFirst(WATER_LOADCELL));
				data[0]= (Hx711_shiftInMsbFirst(WATER_LOADCELL));
				data[2]=~data[2];
				data[1]=~data[1];
				data[0]=~data[0];
				waterLoadCellValue->loadCellMaskValue =(
						(uint32_t)(data[2]) << 16
						| (uint32_t)(data[1]) << 8
						| (uint32_t)(data[0]) );
				// set the channel and the gain factor for the next reading using the clock pin
				for (waterLoadCellValue->loadCellGainSetValue = 0; waterLoadCellValue->loadCellGainSetValue < WATER_LOADCELL->gain; waterLoadCellValue->loadCellGainSetValue++)
				{
					HAL_GPIO_WritePin(WATER_LOADCELL->PD_SCK_PinType, WATER_LOADCELL->PD_SCK_PinNumber, GPIO_PIN_SET);
					HAL_GPIO_WritePin(WATER_LOADCELL->PD_SCK_PinType, WATER_LOADCELL->PD_SCK_PinNumber,GPIO_PIN_RESET);
				}
				sum=waterLoadCellValue->loadCellMaskValue / WATER_LOADCELL->SCALE;
				waterLoadCellValue->actualLoadCellValue = (0x0000FFFF & sum);
				if(sum < Offset_Values_Setting.Water_Loadcell_Offset_Value)
				{
					waterLoadCellValue->loadCellValueAfterTare=0;
				}
				else
				{
					waterLoadCellValue->loadCellValueAfterTare= (sum - Offset_Values_Setting.Water_Loadcell_Offset_Value);
				}
				sum=0;
			}
			Loadcell_Read_state = Oil_Loadcell;
			break;
		}
		osDelay(600);
	}
  /* USER CODE END LOADCELL_PROCESS */
}

/* USER CODE BEGIN Header_TEMPERATURE_CONTROL */
/**
 * @brief Function implementing the Temperature_con thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TEMPERATURE_CONTROL */
void TEMPERATURE_CONTROL(void *argument)
{
  /* USER CODE BEGIN TEMPERATURE_CONTROL */
	/* Infinite loop */
	for(;;)
	{
		Temperature_control(Pressing_Baking.Top_Pan_Cut_off_Temp,Pressing_Baking.Bottom_Pan_Cut_off_Temp);
		osDelay(500);
	}
  /* USER CODE END TEMPERATURE_CONTROL */
}

/* USER CODE BEGIN Header_Android_interface */
/**
 * @brief Function implementing the Android_handler thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Android_interface */
void Android_interface(void *argument)
{
  /* USER CODE BEGIN Android_interface */
	/* Infinite loop */
	for(;;)
	{

		switch(Usb_message_state)
		{
		case Usb_Idle:

			break;
		case Machine_Init:

			break;
		case Flour_dispensing:

			break;
		case Oil_dispensing:

			break;
		case Water_dispensing:

			break;
		case Kneading_IS_Started:

			break;
		case Pressing_is_started:

			break;
		}
		osDelay(5);
	}
  /* USER CODE END Android_interface */
}

/* USER CODE BEGIN Header_StartSpeedControl_Process_Task */
/**
 * @brief Function implementing the SpeedControl_Ta thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSpeedControl_Process_Task */
void StartSpeedControl_Process_Task(void *argument)
{
  /* USER CODE BEGIN StartSpeedControl_Process_Task */
	/* Infinite loop */
	for(;;)
	{
#if FLOUR_PRESENCE_SENSOR
		flourConnectorDistanceinMM = getTOFValue(flourConnectorTOFAddress);
		if(flourConnectorDistanceinMM >= 10)
		{
			errorCntVal.flourConnectorErrorCnt++;
			if(errorCntVal.flourConnectorErrorCnt >= 10)
			{
				processErrorState.flourConnector = 1;
				pnpProximityValues.flourConnectorPresence = 0;
			}
		}
		else if(flourConnectorDistanceinMM < 10 && flourConnectorDistanceinMM > 0)
		{
			pnpProximityValues.flourConnectorPresence = 1;
			errorCntVal.flourConnectorErrorCnt = 0;
			processErrorState.flourConnector = 0;
		}
#endif
		osDelay(300);
	}
  /* USER CODE END StartSpeedControl_Process_Task */
}

/* USER CODE BEGIN Header_Press_Task */
/**
 * @brief Function implementing the Press thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Press_Task */
void Press_Task(void *argument)
{
  /* USER CODE BEGIN Press_Task */

	/* Infinite loop */
	for(;;)
	{
		switch(Press_motor_state)
		{
		case Press_motor_idle:
			press_state_android = 3;
			break;
		case Press_motor_init:
			processErrorState.pressBottom = 0;
			processErrorState.pressTopPos = 0;
			if(machine_initialization_state==machine_initialization_complete)
				press_process_percent = 5;
			press_state_android = 2;
			timerCount.pressTimerCnt = 0;
			pressReverseForceTime = 0;
			press_acl_cnt = 0;
			downDutyCycle = 0;
			Press_motor_state=Press_motor_acceleration_down;
			break;
		case Press_motor_acceleration_down:
			press_acl_cnt++;
			if(press_acl_cnt >= PRESS_MAX_ACCE_DECCELERATE_CNT)
			{
				if(machine_initialization_state==machine_initialization_complete)
				{
					press_process_percent++;
					if(press_process_percent >= 25)
					{
						press_process_percent = 25;
					}
				}
				downDutyCycle++;
				if(downDutyCycle >= 100)
				{
					downDutyCycle = 100;
				}
				if(downDutyCycle >= Pressing_Baking.initial_Press_Motor_Speed)
				{
					downDutyCycle = Pressing_Baking.initial_Press_Motor_Speed;
					timerCount.pressTimerCnt = 0;
					Press_motor_state  = Press_motor_PWM_wait;
				}
				else
				{
					Press_DC_Set_PWM(downDutyCycle, CLOCKWISE);
				}
//				if(LPS_struct.calibratedValueinMM >= (PIZZA_BASE_THICKNESS_CUTOFF_MM-Pressing_Baking.Thickness_Of_pizza))
				if(LPS_struct.calibratedValueinMM >= (((float)(thicknessReferenceValueAndroid - thicknessValueFromAndroid)) - PRESS_STOP_OFFSET))
				{
					Press_DC_Motor_Stop();
					Press_motor_state=press_till_xx_mm_thickness_wait;
				}
				press_acl_cnt = 0;
			}
			break;
		case Press_motor_PWM_wait:
			if(timerCount.pressTimerCnt >= (Pressing_Baking.Intial_Press_Motor_running_time/2))
			{
				if(machine_initialization_state==machine_initialization_complete)
					press_process_percent = 30;
				timerCount.pressTimerCnt = 0;
				Press_motor_state = Press_motor_De_acceleration_down;
			}
			break;
		case Press_motor_De_acceleration_down:
			press_acl_cnt++;
			if(press_acl_cnt >= PRESS_MAX_ACCE_DECCELERATE_CNT)
			{
				if(machine_initialization_state==machine_initialization_complete)
				{
					press_process_percent++;
					if(press_process_percent >= 40)
					{
						press_process_percent = 40;
					}
				}
				downDutyCycle--;
				if(downDutyCycle <= 0)
					downDutyCycle = 0;
				if(downDutyCycle <= Pressing_Baking.Pressing_Force_Duty_cycle)//Pressing_Baking.initial_Press_Motor_Speed)
				{
					timerCount.pressTimerCnt = 0;
					Press_motor_state  = press_till_xx_mm_thickness;
				}
				else
				{
					Press_DC_Set_PWM(downDutyCycle, CLOCKWISE);
				}
				if(LPS_struct.calibratedValueinMM >= (((float)(thicknessReferenceValueAndroid - thicknessValueFromAndroid)) - PRESS_STOP_OFFSET))
				{
					Press_DC_Motor_Stop();
					Press_motor_state=press_till_xx_mm_thickness_wait;
				}
				press_acl_cnt = 0;
			}
			break;
		case press_till_xx_mm_thickness:
			if(machine_initialization_state==machine_initialization_complete)
			{
				press_process_percent++;
				if(press_process_percent >= 55)
				{
					press_process_percent = 55;
				}
			}
			/*29-01-21 Press debounce and forced reverse*/
			pressReverseForceTime++;
			if(LPS_struct.calibratedValueinMM >= PRESS_REVERSE_FORCE_DIST_MM && pressReverseForceTime >= PRESS_REVERSE_FORCE_TIME_MS)
			{
				press_process_percent = 75;
				lpsMinimumtime = 0;
				Press_DC_Motor_Stop();
				timerCount.pressTimerCnt = 0;
				Press_motor_state = Press_motor_homing_init;
			}
			if(LPS_struct.calibratedValueinMM >= (((float)(thicknessReferenceValueAndroid - thicknessValueFromAndroid)) - PRESS_STOP_OFFSET))
			{
				lpsMinimumtime++;
				if(lpsMinimumtime >= LPS_VALUE_DEBOUNCE_COUNT)
				{
					press_process_percent = 60;
					lpsMinimumtime = 0;
					Press_DC_Motor_Stop();
					timerCount.pressTimerCnt = 0;
					Press_motor_state = press_till_xx_mm_thickness_wait;
				}
			}
			if(pressReverseForceTime >= PRESS_REVERSE_FORCE_TIME_MS)
			{
				processErrorState.pressBottom = 1;
				Press_DC_Motor_Stop();
				lpsMinimumtime = 0;
				timerCount.pressTimerCnt = 0;
				press_process_percent = 75;
				Press_motor_state=Press_motor_homing_init;
			}
			/*29-01-21 Press debounce and forced reverse*/
			break;
		case press_till_xx_mm_thickness_wait:
			if(machine_initialization_state==machine_initialization_complete)
			{
				press_process_percent++;
				if(press_process_percent >= 75)
				{
					press_process_percent = 75;
				}
			}
			if(LPS_struct.calibratedValueinMM >= (((float)(thicknessReferenceValueAndroid - thicknessValueFromAndroid)) - PRESS_STOP_OFFSET))
			{
				Press_DC_Motor_Stop();
			}

			if(timerCount.pressTimerCnt >= Pressing_Baking.Holding_Time)
			{
				timerCount.pressTimerCnt= 0;
				pressStopArr[pressInc] = LPS_struct.calibratedValueinMM;
				pressInc++;
				if(pressInc >= 200)
				{
					pressInc = 0;
				}
				Press_motor_state=Press_motor_homing_init;
			}
			break;
		case Press_motor_homing_init:
			if(pnpProximityValues.pressTopPositionLimit==DETECTED)
			{
				Read_ADC2_Channel_state = ADC2_INIT;
				Press_motor_state=Press_motor_homing;
			}
			else
			{
				reverseDutyCycle = 0;
				press_acl_cnt = 0;
				Press_motor_state=Press_motor_acceleration_up;
			}
			break;
		case Press_motor_acceleration_up:
			press_acl_cnt++;
			if(press_acl_cnt >= PRESS_MAX_ACCE_DECCELERATE_CNT)
			{
				if(machine_initialization_state==machine_initialization_complete)
				{
					press_process_percent++;
					if(press_process_percent >= 85)
					{
						press_process_percent = 85;
					}
				}
				reverseDutyCycle++;
				if(reverseDutyCycle >= 100)
				{
					reverseDutyCycle = 100;
				}
				if(Pressing_Baking.Press_Motor_reverse_duty_cycle <= 0)
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 50;
				}
				if(reverseDutyCycle >= Pressing_Baking.Press_Motor_reverse_duty_cycle)
				{
					reverseDutyCycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					reverseWaitTime = 0;
					if(Pressing_Baking.Reverse_Speed_Running_Time <= 0 )
					{
						Pressing_Baking.Reverse_Speed_Running_Time = PRESS_REVERSE_RUNNING_TIME_MS;
					}
					Press_motor_state  = Press_motor_reverse_wait;
				}
				else
				{
					Press_DC_Set_PWM(reverseDutyCycle,ANTICLOCKWISE);
				}
				if(pnpProximityValues.pressTopPositionLimit==DETECTED)
				{
					Press_DC_Motor_Stop();
					Press_motor_state=Press_motor_stop;
				}
				press_acl_cnt = 0;
			}
			break;
		case Press_motor_reverse_wait:
			reverseWaitTime++;
			/*Added proximity Checking for reverse direction on 27-01-2021*/
			if (pnpProximityValues.pressTopPositionLimit != DETECTED)
			{
				if(reverseWaitTime >= (Pressing_Baking.Reverse_Speed_Running_Time / 2))
				{
					press_acl_cnt = 0;
					Press_motor_state=Press_motor_reverse_Force_Slow;
				}
			}
			if(pnpProximityValues.pressTopPositionLimit == DETECTED)
			{
				press_acl_cnt = 0;
				Press_motor_state = Press_motor_stop;
			}
			break;
		case Press_motor_reverse_Force_Slow:
			press_acl_cnt++;
			if(press_acl_cnt >= PRESS_MAX_ACCE_DECCELERATE_CNT)
			{
				if(machine_initialization_state==machine_initialization_complete)
				{
					press_process_percent++;
					if(press_process_percent >= 85)
					{
						press_process_percent = 85;
					}
				}
				reverseDutyCycle--;
				if(reverseDutyCycle >= 100)
				{
					reverseDutyCycle = 100;
				}
				if(Pressing_Baking.Reverse_Force_Duty_Cycle <= 0)
				{
					Pressing_Baking.Reverse_Force_Duty_Cycle = 50;
				}
				if(reverseDutyCycle <= Pressing_Baking.Reverse_Force_Duty_Cycle)
				{
					reverseDutyCycle = Pressing_Baking.Reverse_Force_Duty_Cycle;
					Press_motor_state  = Press_motor_homing;
				}
				else
				{
					Press_DC_Set_PWM(reverseDutyCycle,ANTICLOCKWISE);
				}
				if(pnpProximityValues.pressTopPositionLimit==DETECTED)
				{
					Press_DC_Motor_Stop();
					Press_motor_state=Press_motor_stop;
				}
				press_acl_cnt = 0;
			}
			break;
		case Press_motor_homing:
			if(pnpProximityValues.pressTopPositionLimit==DETECTED)
			{
				if(machine_initialization_state==machine_initialization_complete)
				{
					press_process_percent = 90;
				}
				Read_ADC2_Channel_state = ADC2_INIT;
				pressReverseForceTime = 0;
				Press_DC_Motor_Stop();
				Press_motor_state=Press_motor_stop;
			}
			if(pressTopPositionErrorCnt++ >= PRESS_TOP_POS_ERROR_TIME_MS)
			{
				processErrorState.pressTopPos = 1;
//				Press_DC_Motor_Stop();
			}
			break;
		case Press_motor_stop:
			if(machine_initialization_state==machine_initialization_complete)
			{
				press_process_percent = 90;
			}
			pressForcePWM = 0;
			initialPressPWM = 30;
			Press_DC_Motor_Stop();
			Press_motor_state=Press_motor_completed;
			break;
		case Press_motor_completed:
			if(machine_initialization_state == machine_initialization_complete)
			{
				press_process_percent = 100;
			}
			press_state_android = 3;
			Press_DC_Motor_Stop();
			break;
		}
		/*Added on 16-Feb for Cleaning position of Press*/
		switch(Press_motor_cleaning_bottom)
		{
			case Press_motor_idle:
				break;
			case Press_motor_init:
				timerCount.pressTimerCnt = 0;
				press_acl_cnt = 0;
				downDutyCycle = 0;
				Press_motor_cleaning_bottom=Press_motor_acceleration_down;
				break;
			case Press_motor_acceleration_down:
				press_acl_cnt++;
				if(press_acl_cnt >= PRESS_MAX_ACCE_DECCELERATE_CNT)
				{
					downDutyCycle++;
					if(downDutyCycle >= 50)
					{
						downDutyCycle = 50;
					}
					if(downDutyCycle >= PRESS_DUTYCYCLE_CLEANING)
					{
						downDutyCycle = PRESS_DUTYCYCLE_CLEANING;
						Press_DC_Set_PWM(downDutyCycle, CLOCKWISE);
						timerCount.pressTimerCnt = 0;
						Press_motor_cleaning_bottom  = press_till_xx_mm_thickness;
					}
					else
					{
						Press_DC_Set_PWM(downDutyCycle, CLOCKWISE);
					}
					if(LPS_struct.calibratedValueinMM >= (PIZZA_BASE_THICKNESS_CUTOFF_MM - PRESS_DEFAULT_THICKNESS))
					{
						Press_DC_Motor_Stop();
						Press_motor_cleaning_bottom=Press_motor_completed;
					}
					press_acl_cnt = 0;
				}
				break;
			case press_till_xx_mm_thickness:
				if(LPS_struct.calibratedValueinMM >= (PIZZA_BASE_THICKNESS_CUTOFF_MM - PRESS_DEFAULT_THICKNESS))
				{
					lpsMinimumtime++;
					if(lpsMinimumtime >= LPS_VALUE_DEBOUNCE_COUNT)
					{
						press_process_percent = 60;
						lpsMinimumtime = 0;
						Press_DC_Motor_Stop();
						timerCount.pressTimerCnt = 0;
						Press_motor_cleaning_bottom = Press_motor_completed;
					}
				}
				if(pressReverseForceTime++ >= PRESS_REVERSE_FORCE_TIME_MS)
				{
					processErrorState.pressBottom = 1;
					Press_DC_Motor_Stop();
					lpsMinimumtime = 0;
					timerCount.pressTimerCnt = 0;
				}
				break;
			case Press_motor_completed:
				 break;
		}
		/*Added on 16-Feb for Cleaning position of Press*/
		osDelay(1);
	}
  /* USER CODE END Press_Task */
}

/* USER CODE BEGIN Header_Dispensing_Process_task */
/**
 * @brief Function implementing the Dispensing_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Dispensing_Process_task */
void Dispensing_Process_task(void *argument)
{
  /* USER CODE BEGIN Dispensing_Process_task */
	/*
#if !androidParameter
	Flour_Stepper_Motor.rpm=FLOUR_STEPPER_DEFAULT_RPM;
	Water_Stepper_Motor.rpm=WATER_STEPPER_DEFAULT_RPM;
	Oil_Stepper_Motor.rpm=OIL_STEPPER_DEFAULT_RPM;
#endif
	 */
	Flour_Stepper_Motor.timer_clock=STEPPER_TIMER_CLOCK;
	Flour_Stepper_Motor.step_angle=FLOUR_STEPPER_STEP;
	Flour_Stepper_Motor.Period=STEPPER_TIMER_PERIOD;
	Flour_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = HW_Setting1.Flour_Flow_rate_at_level_5;//10;
//	Flour_Stepper_Motor.direction = CLOCKWISE;
	Oil_Stepper_Motor.timer_clock=STEPPER_TIMER_CLOCK;
	Oil_Stepper_Motor.step_angle=STEPPER_STEP;
	Oil_Stepper_Motor.Period=STEPPER_TIMER_PERIOD;
	Oil_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = HW_Setting1.Oil_Pump_Flow_Rate;
	//	Oil_Stepper_Motor.direction = ANTICLOCKWISE;
	Water_Stepper_Motor.timer_clock=STEPPER_TIMER_CLOCK;
	Water_Stepper_Motor.step_angle=STEPPER_STEP;
	Water_Stepper_Motor.Period=STEPPER_TIMER_PERIOD;
	Water_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = HW_Setting1.Water_Pump_Flow_Rate;
	//	Water_Stepper_Motor.direction = ANTICLOCKWISE;

	Loadcell_Read_state = Flour_loadcell;

	/* Infinite loop */
	for(;;)
	{
		if(Water_stepper_motor_state == Water_stepper_motor_wait_to_complete)
		{
			errorCntVal.waterDispense++;
			if(errorCntVal.waterDispense >= 2000 && (water_pulse_count <= 10))
			{
				processErrorState.waterFlow = 1;
//				HAL_TIM_Base_Stop_IT(&htim7);
//				Water_Stepper_Motor.rpm_counter=0;
//				Water_Stepper_Motor.steps_counter=0;
//				Water_Stepper_Motor.total_no_of_steps =0;
//				HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port,WATER_DISP_EN_Pin,RESET);
//				HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin,RESET);
			}
		}
		switch(Oil_stepper_motor_state)
		{
		case Oil_stepper_motor_idle:

			break;
		case Oil_stepper_motor_init:
			Loadcell_Read_state=Oil_Loadcell;
			dispenseStateForInterrupt = oilDispense;
			Oil_stepper_motor_state = Oil_stepper_motor_start;
			break;
		case Oil_stepper_motor_start:
			HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin, SET);
			if(Oil_Stepper_Motor.direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(OIL_DISP_DIR_GPIO_Port, OIL_DISP_DIR_Pin, SET);
			}
			else if(Oil_Stepper_Motor.direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(OIL_DISP_DIR_GPIO_Port, OIL_DISP_DIR_Pin, RESET);
			}
			Oil_Stepper_Motor.steps_counter = 0;
			HAL_TIM_Base_Start_IT(&htim7);
			if(Oil_Stepper_Motor.rpm > 100)
			{
				Oil_Stepper_Motor.rpm_counter = 100;
			}
			Oil_stepper_motor_state = oil_stepper_motor_acceleration;
			break;
		case oil_stepper_motor_acceleration:
			Oil_Stepper_Motor.rpm_counter++;
			if(Oil_Stepper_Motor.rpm_counter<Oil_Stepper_Motor.rpm)
			{
				Oil_Stepper_Motor.frequency=(uint32_t)(((Oil_Stepper_Motor.rpm*360)/(Oil_Stepper_Motor.step_angle*60))*2);
				Oil_Stepper_Motor.prescaler=(uint32_t)(Oil_Stepper_Motor.timer_clock/(Oil_Stepper_Motor.Period*Oil_Stepper_Motor.frequency));
				TIM7->ARR=(Oil_Stepper_Motor.Period-1);
				TIM7->PSC=(Oil_Stepper_Motor.prescaler-1);
			}
			else
			{
				Oil_Stepper_Motor.rpm_counter=0;
				Oil_stepper_motor_state=Oil_stepper_motor_wait_to_complete;
			}
			if(( Oil_Stepper_Motor.steps_counter >= Oil_Stepper_Motor.total_no_of_steps))
			{
				HAL_TIM_Base_Stop_IT(&htim7);
				Oil_Stepper_Motor.rpm_counter=0;
				Oil_Stepper_Motor.steps_counter=0;
				Oil_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);
				Oil_stepper_motor_state = Oil_stepper_motor_completed;
			}
			break;
		case 	Oil_stepper_motor_wait_to_complete:
			if(( Oil_Stepper_Motor.steps_counter >= Oil_Stepper_Motor.total_no_of_steps))
			{
				HAL_TIM_Base_Stop_IT(&htim7);
				Oil_Stepper_Motor.rpm_counter=0;
				Oil_Stepper_Motor.steps_counter=0;
				Oil_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);
				Oil_stepper_motor_state = Oil_stepper_motor_completed;
			}
			break;
		case Oil_stepper_motor_completed:
			HAL_TIM_Base_Stop_IT(&htim7);
			Oil_Stepper_Motor.rpm_counter=0;
			Oil_Stepper_Motor.steps_counter=0;
			Oil_Stepper_Motor.total_no_of_steps =0;
			HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
			HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);
			break;
		}
		switch(Water_stepper_motor_state)
		{
		case Water_stepper_motor_idle:

			break;
		case Water_stepper_motor_init:
			Loadcell_Read_state=Water_Loadcell;
			dispenseStateForInterrupt = waterDispense;
			Water_stepper_motor_state = Water_stepper_motor_start;
			break;
		case Water_stepper_motor_start:
			HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port, WATER_DISP_EN_Pin, SET);
			if(Water_Stepper_Motor.direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(WATER_DISP_DIR_GPIO_Port, WATER_DISP_DIR_Pin, SET);
			}
			else if(Water_Stepper_Motor.direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(WATER_DISP_DIR_GPIO_Port, WATER_DISP_DIR_Pin, RESET);
			}
			Water_Stepper_Motor.steps_counter = 0;
			if(Water_Stepper_Motor.rpm > 100)
			{
				Water_Stepper_Motor.rpm_counter = 100;
			}
			HAL_TIM_Base_Start_IT(&htim7);
			Water_stepper_motor_state = Water_stepper_motor_acceleration;
			break;
		case Water_stepper_motor_acceleration:
			Water_Stepper_Motor.rpm_counter++;
			if(Water_Stepper_Motor.rpm_counter<Water_Stepper_Motor.rpm)
			{
				Water_Stepper_Motor.frequency=(uint32_t)(((Water_Stepper_Motor.rpm*360)/(Water_Stepper_Motor.step_angle*60))*2);
				Water_Stepper_Motor.prescaler=(uint32_t)(Water_Stepper_Motor.timer_clock/(Water_Stepper_Motor.Period*Water_Stepper_Motor.frequency));
				TIM7->ARR=(Water_Stepper_Motor.Period-1);
				TIM7->PSC=(Water_Stepper_Motor.prescaler-1);
			}
			else
			{
				Water_Stepper_Motor.rpm_counter=0;
				Water_stepper_motor_state=Water_stepper_motor_wait_to_complete;
			}
			if((water_pulse_count >= (water_value) && (machine_initialization_state == machine_initialization_complete))||( Water_Stepper_Motor.steps_counter >= (Water_Stepper_Motor.total_no_of_steps + WATER_DISPENSE_STEPS_TOLERANCE)) )
				//			if(( Water_Stepper_Motor.steps_counter >= Water_Stepper_Motor.total_no_of_steps))
			{
				water_pulse_count = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				Water_Stepper_Motor.rpm_counter=0;
				Water_Stepper_Motor.steps_counter=0;
				Water_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port,WATER_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin,RESET);
				Water_stepper_motor_state = Water_stepper_motor_completed;
			}
			break;
		case 	Water_stepper_motor_wait_to_complete:
			if((water_pulse_count >= (water_value) && (machine_initialization_state == machine_initialization_complete))||( Water_Stepper_Motor.steps_counter >= (Water_Stepper_Motor.total_no_of_steps + WATER_DISPENSE_STEPS_TOLERANCE)) )
				//			if(( Water_Stepper_Motor.steps_counter >= Water_Stepper_Motor.total_no_of_steps))
			{
				water_pulse_count = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				Water_Stepper_Motor.rpm_counter=0;
				Water_Stepper_Motor.steps_counter=0;
				Water_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port,WATER_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin,RESET);
				Water_stepper_motor_state = Water_stepper_motor_completed;
			}
			break;
		case Water_stepper_motor_completed:
			break;
		}

		switch(Flour_stepper_motor_state)
		{
		case Flour_stepper_motor_idle:
			break;
		case Flour_stepper_motor_init:
			Loadcell_Read_state=Flour_loadcell;
			dispenseStateForInterrupt = flourDispense;
			Flour_stepper_motor_state = Flour_stepper_motor_start;
			break;
		case Flour_stepper_motor_start:
			HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin, SET);
			if(Flour_Stepper_Motor.direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(FLOUR_DISP_DIR_GPIO_Port, FLOUR_DISP_DIR_Pin, SET);
			}
			else if(Flour_Stepper_Motor.direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(FLOUR_DISP_DIR_GPIO_Port, FLOUR_DISP_DIR_Pin, RESET);
			}
			HAL_GPIO_WritePin(FLOUR_DISP_DIR_GPIO_Port, FLOUR_DISP_DIR_Pin, RESET);
			Flour_Stepper_Motor.steps_counter = 0;
			HAL_TIM_Base_Start_IT(&htim7);
			Flour_stepper_motor_state = Flour_stepper_motor_acceleration;
			break;
		case Flour_stepper_motor_acceleration:
			Flour_Stepper_Motor.rpm_counter++;
			if(Flour_Stepper_Motor.rpm_counter<Flour_Stepper_Motor.rpm)
			{
				Flour_Stepper_Motor.frequency=(uint32_t)(((Flour_Stepper_Motor.rpm*360)/(Flour_Stepper_Motor.step_angle*60))*2);
				Flour_Stepper_Motor.prescaler=(uint32_t)(Flour_Stepper_Motor.timer_clock/(Flour_Stepper_Motor.Period*Flour_Stepper_Motor.frequency));
				TIM7->ARR=(Flour_Stepper_Motor.Period-1);
				TIM7->PSC=(Flour_Stepper_Motor.prescaler-1);
			}
			else
			{
				Flour_Stepper_Motor.rpm_counter=0;
				Flour_stepper_motor_state=Flour_stepper_motor_wait_to_complete;
			}
			if(( Flour_Stepper_Motor.steps_counter >= Flour_Stepper_Motor.total_no_of_steps))
			{
				HAL_TIM_Base_Stop_IT(&htim7);
				Flour_Stepper_Motor.rpm_counter=0;
				Flour_Stepper_Motor.steps_counter=0;
				Flour_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin,RESET);
				Flour_stepper_motor_state = Flour_stepper_motor_completed;
			}
			break;
		case 	Flour_stepper_motor_wait_to_complete:
			if(( Flour_Stepper_Motor.steps_counter >= (Flour_Stepper_Motor.total_no_of_steps)))		//Added *2 since each pulse is taken as one step. To get required steps need to add high & low. Hence multiply by 2
			{
				HAL_TIM_Base_Stop_IT(&htim7);
				Flour_Stepper_Motor.rpm_counter=0;
				Flour_Stepper_Motor.steps_counter=0;
				Flour_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin,RESET);
				Flour_stepper_motor_state = Flour_stepper_motor_completed;
			}
			break;
		case Flour_stepper_motor_completed:
			Agitator_MTR_Stop();
			HAL_TIM_Base_Stop_IT(&htim7);
			Flour_Stepper_Motor.rpm_counter=0;
			Flour_Stepper_Motor.steps_counter=0;
			Flour_Stepper_Motor.total_no_of_steps =0;
			HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin,RESET);
			HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin,RESET);
			break;
		}


		/*Maintenance Mode Dispensing  added on 15-Mar*/
		switch(Oil_stepper_maintenance_state)
		{
		case Oil_stepper_motor_idle:

			break;
		case Oil_stepper_motor_init:
			Loadcell_Read_state=Oil_Loadcell;
			dispenseStateForInterrupt = oilDispense;
			Oil_stepper_maintenance_state = Oil_stepper_motor_start;
			break;
		case Oil_stepper_motor_start:
			HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin, SET);
			if(Oil_Stepper_Motor.direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(OIL_DISP_DIR_GPIO_Port, OIL_DISP_DIR_Pin, SET);
			}
			else if(Oil_Stepper_Motor.direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(OIL_DISP_DIR_GPIO_Port, OIL_DISP_DIR_Pin, RESET);
			}
			Oil_Stepper_Motor.steps_counter = 0;
			HAL_TIM_Base_Start_IT(&htim7);
			Oil_stepper_maintenance_state = oil_stepper_motor_acceleration;
			break;
		case oil_stepper_motor_acceleration:
			Oil_Stepper_Motor.rpm_counter++;
			if(Oil_Stepper_Motor.rpm_counter<Oil_Stepper_Motor.rpm)
			{
				Oil_Stepper_Motor.frequency=(uint32_t)(((Oil_Stepper_Motor.rpm*360)/(Oil_Stepper_Motor.step_angle*60))*2);
				Oil_Stepper_Motor.prescaler=(uint32_t)(Oil_Stepper_Motor.timer_clock/(Oil_Stepper_Motor.Period*Oil_Stepper_Motor.frequency));
				TIM7->ARR=(Oil_Stepper_Motor.Period-1);
				TIM7->PSC=(Oil_Stepper_Motor.prescaler-1);
			}
			else
			{
				Oil_Stepper_Motor.rpm_counter=0;
				Oil_stepper_maintenance_state=Oil_stepper_motor_wait_to_complete;
			}
			if(timerCount.oilMaintenance >= oilRunTimeFromAndroid)
			{
				HAL_TIM_Base_Stop_IT(&htim7);
				Oil_Stepper_Motor.rpm_counter=0;
				Oil_Stepper_Motor.steps_counter=0;
				Oil_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);
				Oil_stepper_maintenance_state = Oil_stepper_motor_completed;
			}
			break;
		case 	Oil_stepper_motor_wait_to_complete:
			if(( timerCount.oilMaintenance >= oilRunTimeFromAndroid))
			{
				HAL_TIM_Base_Stop_IT(&htim7);
				Oil_Stepper_Motor.rpm_counter=0;
				Oil_Stepper_Motor.steps_counter=0;
				Oil_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);
				Oil_stepper_maintenance_state = Oil_stepper_motor_completed;
			}
			break;
		case Oil_stepper_motor_completed:
			HAL_TIM_Base_Stop_IT(&htim7);
			Oil_Stepper_Motor.rpm_counter=0;
			Oil_Stepper_Motor.steps_counter=0;
			Oil_Stepper_Motor.total_no_of_steps =0;
			HAL_GPIO_WritePin(OIL_DISP_EN_GPIO_Port, OIL_DISP_EN_Pin,RESET);
			HAL_GPIO_WritePin(OIL_DISP_PULSE_GPIO_Port, OIL_DISP_PULSE_Pin,RESET);
			break;
		}
		switch(Water_stepper_maintenance_state)
		{
		case Water_stepper_motor_idle:

			break;
		case Water_stepper_motor_init:
			Loadcell_Read_state=Water_Loadcell;
			dispenseStateForInterrupt = waterDispense;
			Water_stepper_maintenance_state = Water_stepper_motor_start;
			break;
		case Water_stepper_motor_start:
			HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port, WATER_DISP_EN_Pin, SET);
			if(Water_Stepper_Motor.direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(WATER_DISP_DIR_GPIO_Port, WATER_DISP_DIR_Pin, SET);
			}
			else if(Water_Stepper_Motor.direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(WATER_DISP_DIR_GPIO_Port, WATER_DISP_DIR_Pin, RESET);
			}
			Water_Stepper_Motor.steps_counter = 0;
			HAL_TIM_Base_Start_IT(&htim7);
			Water_stepper_maintenance_state = Water_stepper_motor_acceleration;
			break;
		case Water_stepper_motor_acceleration:
			Water_Stepper_Motor.rpm_counter++;
			if(Water_Stepper_Motor.rpm_counter<Water_Stepper_Motor.rpm)
			{
				Water_Stepper_Motor.frequency=(uint32_t)(((Water_Stepper_Motor.rpm*360)/(Water_Stepper_Motor.step_angle*60))*2);
				Water_Stepper_Motor.prescaler=(uint32_t)(Water_Stepper_Motor.timer_clock/(Water_Stepper_Motor.Period*Water_Stepper_Motor.frequency));
				TIM7->ARR=(Water_Stepper_Motor.Period-1);
				TIM7->PSC=(Water_Stepper_Motor.prescaler-1);
			}
			else
			{
				Water_Stepper_Motor.rpm_counter=0;
				Water_stepper_maintenance_state=Water_stepper_motor_wait_to_complete;
			}
			if(timerCount.waterMaintenance >= waterRunTimeFromAndroid)
			{
				water_pulse_count = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				Water_Stepper_Motor.rpm_counter=0;
				Water_Stepper_Motor.steps_counter=0;
				Water_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port,WATER_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin,RESET);
				Water_stepper_maintenance_state = Water_stepper_motor_completed;
			}
			break;
		case 	Water_stepper_motor_wait_to_complete:
			if(timerCount.waterMaintenance >= waterRunTimeFromAndroid)
			{
				water_pulse_count = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				Water_Stepper_Motor.rpm_counter=0;
				Water_Stepper_Motor.steps_counter=0;
				Water_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(WATER_DISP_EN_GPIO_Port,WATER_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(WATER_DISP_PULSE_GPIO_Port, WATER_DISP_PULSE_Pin,RESET);
				Water_stepper_maintenance_state = Water_stepper_motor_completed;
			}
			break;
		case Water_stepper_motor_completed:
			break;
		}
		switch(Flour_stepper_maintenance_state)
		{
		case Flour_stepper_motor_idle:

			break;
		case Flour_stepper_motor_init:
			Loadcell_Read_state=Flour_loadcell;
			dispenseStateForInterrupt = flourDispense;
			Flour_stepper_maintenance_state = Flour_stepper_motor_start;
			break;
		case Flour_stepper_motor_start:
			HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin, SET);
			if(Flour_Stepper_Motor.direction == ANTICLOCKWISE)
			{
				HAL_GPIO_WritePin(FLOUR_DISP_DIR_GPIO_Port, FLOUR_DISP_DIR_Pin, SET);
			}
			else if(Flour_Stepper_Motor.direction == CLOCKWISE)
			{
				HAL_GPIO_WritePin(FLOUR_DISP_DIR_GPIO_Port, FLOUR_DISP_DIR_Pin, RESET);
			}
			Flour_Stepper_Motor.steps_counter = 0;
			HAL_TIM_Base_Start_IT(&htim7);
			Flour_stepper_maintenance_state = Flour_stepper_motor_acceleration;
			break;
		case Flour_stepper_motor_acceleration:
			Flour_Stepper_Motor.rpm_counter++;
			if(Flour_Stepper_Motor.rpm_counter<Flour_Stepper_Motor.rpm)
			{
				Flour_Stepper_Motor.frequency=(uint32_t)(((Flour_Stepper_Motor.rpm*360)/(Flour_Stepper_Motor.step_angle*60))*2);
				Flour_Stepper_Motor.prescaler=(uint32_t)(Flour_Stepper_Motor.timer_clock/(Flour_Stepper_Motor.Period*Flour_Stepper_Motor.frequency));
				TIM7->ARR=(Flour_Stepper_Motor.Period-1);
				TIM7->PSC=(Flour_Stepper_Motor.prescaler-1);
			}
			else
			{
				Flour_Stepper_Motor.rpm_counter=0;
				Flour_stepper_maintenance_state=Flour_stepper_motor_wait_to_complete;
			}
			break;
		case 	Flour_stepper_motor_wait_to_complete:
			if(( timerCount.flourMaintenance >= flourRunTimeFromAndroid))
			{
				timerCount.flourMaintenance =0;
				HAL_TIM_Base_Stop_IT(&htim7);
				Flour_Stepper_Motor.rpm_counter=0;
				Flour_Stepper_Motor.steps_counter=0;
				Flour_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin,RESET);
				HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin,RESET);
				Flour_stepper_maintenance_state = Flour_stepper_motor_completed;
			}
			break;
		case Flour_stepper_motor_completed:
			HAL_TIM_Base_Stop_IT(&htim7);
			Flour_Stepper_Motor.rpm_counter=0;
			Flour_Stepper_Motor.steps_counter=0;
			Flour_Stepper_Motor.total_no_of_steps =0;
			HAL_GPIO_WritePin(FLOUR_DISP_EN_GPIO_Port, FLOUR_DISP_EN_Pin,RESET);
			HAL_GPIO_WritePin(FLOUR_DISP_PULSE_GPIO_Port, FLOUR_DISP_PULSE_Pin,RESET);
			break;
		}
		/*Maintenance Mode Dispensing added on 15-Mar*/
		switch(Agitator_motor_state)
		{
		case Agitator_motor_idle:

			break;
		case Agitator_init:
			Agitator_MTR_Run(80.0, CLOCKWISE);
			Agitator_motor_state=Agitator_motor_start;
			break;
		case Agitator_motor_start:
			Agitator_MTR_Run(80.0, CLOCKWISE);
			break;
		case Agitator_motor_stop:
			Agitator_MTR_Stop();
			break;
		case Agitator_motor_completed:
			Agitator_MTR_Stop();
			break;
		}
		osDelay(1);
	}
  /* USER CODE END Dispensing_Process_task */
}

/* USER CODE BEGIN Header_Kneading_Motor_task */
/**
* @brief Function implementing the Kneading_Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Kneading_Motor_task */
void Kneading_Motor_task(void *argument)
{
  /* USER CODE BEGIN Kneading_Motor_task */
	Knead_screw_Stepper_Motor.timer_clock=STEPPER_TIMER_CLOCK;
	Knead_screw_Stepper_Motor.step_angle=STEPPER_STEP;
	Knead_screw_Stepper_Motor.Period=STEPPER_TIMER_PERIOD;
	Knead_screw_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = KNEADER_LEADSCREW_STEPS_PER_MM;  //Steps per mm(1 rotation = 5mm) * 2(for high and low pulse)

	/* Infinite loop */
	for(;;)
	{
		if(knead_screw_homing_or_top_end_limit_state == knead_screw_wait_to_complete)
		{
			errorCntVal.leadscrewErrorCnt++;
			if(errorCntVal.leadscrewErrorCnt >= KNEADER_LEADSCREW_ERROR_TIME)
			{
				errorCntVal.leadscrewErrorCnt = 0;
				processErrorState.kneaderLeadscrewTop = 1;
//				HAL_TIM_Base_Stop_IT(&htim14);
//				knead_screw_homing_or_top_end_limit_state = knead_screw_idle;
//				knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
//				Knead_screw_Stepper_Motor.rpm_counter=0;
//				Knead_screw_Stepper_Motor.steps_counter=0;
//				Knead_screw_Stepper_Motor.total_no_of_steps =0;
//				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
//				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
			}
		}
		else if(knead_screw_rear_end_or_bottom_end_limit_state == knead_screw_wait_to_complete)
		{
			errorCntVal.leadscrewErrorCnt++;
			if(errorCntVal.leadscrewErrorCnt >= KNEADER_LEADSCREW_ERROR_TIME)
			{
				errorCntVal.leadscrewErrorCnt = 0;
				processErrorState.kneaderLeadscrewBottom = 1;
//				HAL_TIM_Base_Stop_IT(&htim14);
//				knead_screw_homing_or_top_end_limit_state = knead_screw_idle;
//				knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_idle;
//				Knead_screw_Stepper_Motor.rpm_counter=0;
//				Knead_screw_Stepper_Motor.steps_counter=0;
//				Knead_screw_Stepper_Motor.total_no_of_steps =0;
//				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
//				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
			}
		}
		/*if(errorCntVal.leadscrewErrorCnt++ >= KNEADER_LEADSCREW_ERROR_TIME)
		{
			  if(knead_screw_homing_or_top_end_limit_state != knead_screw_completed && knead_screw_homing_or_top_end_limit_state != knead_screw_idle)
			  {
				  knead_screw_homing_or_top_end_limit_state = knead_screw_start;
				  if(errorRetryVal.leadscrew++ >= KNEADER_LEADSCREW_RETRY_MAX_COUNT)
				  {
					  knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
					  errorRetryVal.leadscrew = 0;
				  }
				  errorCntVal.leadscrewErrorCnt = 0;
			  }
			  else if(knead_screw_rear_end_or_bottom_end_limit_state != knead_screw_completed && knead_screw_rear_end_or_bottom_end_limit_state != knead_screw_idle)
			  {
				  knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
				  if(errorRetryVal.leadscrew++ >= KNEADER_LEADSCREW_RETRY_MAX_COUNT)
				  {
					  knead_screw_homing_or_top_end_limit_state = knead_screw_start;
					  errorRetryVal.leadscrew = 0;
				  }
				  errorCntVal.leadscrewErrorCnt = 0;
			  }
			  else if(Knead_movement_to_xx_mm_state != knead_screw_completed && Knead_movement_to_xx_mm_state != knead_screw_idle)
			  {
				  knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
				  errorCntVal.leadscrewErrorCnt = 0;
			  }
		}*/
		switch(knead_screw_homing_or_top_end_limit_state)
		{
		case knead_screw_idle:
			break;
		case knead_screw_start:
			errorCntVal.leadscrewErrorCnt = 0;
			if(pnpProximityValues.leadscrewTopPositionLimit == DETECTED)
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_completed;
			}
			else
			{
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, SET);
				HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port,LEADSCREW_DIR_Pin, SET);
				if(Knead_screw_Stepper_Motor.rpm > 100)
				{
					Knead_screw_Stepper_Motor.rpm_counter = 100;
				}
				HAL_TIM_Base_Start_IT(&htim14);
				Knead_screw_Stepper_Motor.direction = CLOCKWISE;
				knead_screw_homing_or_top_end_limit_state=knead_scerw_acceration;
			}
			break;
		case knead_scerw_acceration:
			Knead_screw_Stepper_Motor.rpm_counter++;
			if(Knead_screw_Stepper_Motor.rpm_counter<Knead_screw_Stepper_Motor.rpm)
			{
				Knead_screw_Stepper_Motor.frequency=(uint32_t)((Knead_screw_Stepper_Motor.rpm_counter*360)/(Knead_screw_Stepper_Motor.step_angle*60))*2;
				Knead_screw_Stepper_Motor.prescaler=(uint32_t)(Knead_screw_Stepper_Motor.timer_clock/(Knead_screw_Stepper_Motor.Period*Knead_screw_Stepper_Motor.frequency));
				TIM14->ARR=(Knead_screw_Stepper_Motor.Period-1);
				TIM14->PSC=(Knead_screw_Stepper_Motor.prescaler-1);
			}
			else
			{
				Knead_screw_Stepper_Motor.rpm_counter=0;
				knead_screw_homing_or_top_end_limit_state=ejecter_wait_to_complete;
			}
			break;
		case knead_screw_wait_to_complete:
			if(pnpProximityValues.leadscrewTopPositionLimit == DETECTED)
			{
				errorCntVal.leadscrewErrorCnt = 0;
				HAL_TIM_Base_Stop_IT(&htim14);
				Knead_screw_Stepper_Motor.rpm_counter=0;
				Knead_screw_Stepper_Motor.steps_counter=0;
				Knead_screw_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
				kneadLeadscrewTravel.previousMMTravel = 0;
				knead_screw_homing_or_top_end_limit_state=knead_screw_completed;
			}
			break;
		case knead_screw_completed:
//			HAL_TIM_Base_Stop_IT(&htim14);
//			Knead_screw_Stepper_Motor.rpm_counter=0;
//			Knead_screw_Stepper_Motor.steps_counter=0;
//			Knead_screw_Stepper_Motor.total_no_of_steps =0;
//			HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
//			HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
//			knead_screw_homing_or_top_end_limit_state=knead_screw_idle;
//			kneadLeadscrewTravel.previousMMTravel=0;
			break;
		}
		switch(Knead_movement_to_xx_mm_state)
		{
		case knead_screw_idle:
			break;

		case knead_screw_start:
			errorCntVal.leadscrewErrorCnt = 0;
			if(Knead_screw_Stepper_Motor.rpm > 100)
			{
				Knead_screw_Stepper_Motor.rpm_counter = 100;
			}
			if((kneadLeadscrewTravel.newMMTravel > kneadLeadscrewTravel.previousMMTravel)&&(kneadLeadscrewTravel.newMMTravel <= kneadLeadscrewTravel.maximumMMTravel))
			{
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, SET);
				HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port,LEADSCREW_DIR_Pin, RESET);
				Knead_screw_Stepper_Motor.direction = ANTICLOCKWISE;
				Knead_screw_Stepper_Motor.total_no_of_steps = kneadLeadscrewTravel.newMMTravel-kneadLeadscrewTravel.previousMMTravel;
				Knead_screw_Stepper_Motor.total_no_of_steps*=Knead_screw_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml;
				HAL_TIM_Base_Start_IT(&htim14);
				Knead_movement_to_xx_mm_state=knead_scerw_acceration;
				kneadLeadscrewTravel.previousMMTravel=kneadLeadscrewTravel.newMMTravel;
			}
			else if((kneadLeadscrewTravel.newMMTravel < kneadLeadscrewTravel.previousMMTravel)&&(kneadLeadscrewTravel.newMMTravel <= kneadLeadscrewTravel.maximumMMTravel))
			{
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, SET);
				HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port,LEADSCREW_DIR_Pin, SET);
				Knead_screw_Stepper_Motor.direction = CLOCKWISE;
				Knead_screw_Stepper_Motor.total_no_of_steps = kneadLeadscrewTravel.previousMMTravel-kneadLeadscrewTravel.newMMTravel;
				Knead_screw_Stepper_Motor.total_no_of_steps*=Knead_screw_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml;
				HAL_TIM_Base_Start_IT(&htim14);
				Knead_movement_to_xx_mm_state=knead_scerw_acceration;
				kneadLeadscrewTravel.previousMMTravel=kneadLeadscrewTravel.newMMTravel;
			}
			else
			{
				Knead_movement_to_xx_mm_state=knead_screw_completed;
			}
			break;
		case knead_scerw_acceration:
			Knead_screw_Stepper_Motor.rpm_counter++;
			if(Knead_screw_Stepper_Motor.rpm_counter<Knead_screw_Stepper_Motor.rpm)
			{
				Knead_screw_Stepper_Motor.frequency=(uint32_t)((Knead_screw_Stepper_Motor.rpm_counter*360)/(Knead_screw_Stepper_Motor.step_angle*60))*2;
				Knead_screw_Stepper_Motor.prescaler=(uint32_t)(Knead_screw_Stepper_Motor.timer_clock/(Knead_screw_Stepper_Motor.Period*Knead_screw_Stepper_Motor.frequency));
				TIM14->ARR=(Knead_screw_Stepper_Motor.Period-1);
				TIM14->PSC=(Knead_screw_Stepper_Motor.prescaler-1);
			}
			else
			{
				Knead_screw_Stepper_Motor.rpm_counter=0;
				Knead_movement_to_xx_mm_state=knead_screw_wait_to_complete;
			}
			if((Knead_screw_Stepper_Motor.direction == ANTICLOCKWISE)&&((Knead_screw_Stepper_Motor.steps_counter>=Knead_screw_Stepper_Motor.total_no_of_steps) || (pnpProximityValues.leadscrewBottomPositionLimit == DETECTED)))
			{
				HAL_TIM_Base_Stop_IT(&htim14);
				Knead_screw_Stepper_Motor.rpm_counter=0;
				Knead_screw_Stepper_Motor.steps_counter=0;
				Knead_screw_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
				Knead_movement_to_xx_mm_state=knead_screw_completed;
			}
			else if((Knead_screw_Stepper_Motor.direction == CLOCKWISE) && ((Knead_screw_Stepper_Motor.steps_counter>=Knead_screw_Stepper_Motor.total_no_of_steps) || (pnpProximityValues.leadscrewTopPositionLimit == DETECTED)))
			{
				HAL_TIM_Base_Stop_IT(&htim14);
				Knead_screw_Stepper_Motor.rpm_counter=0;
				Knead_screw_Stepper_Motor.steps_counter=0;
				Knead_screw_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
				Knead_movement_to_xx_mm_state=knead_screw_completed;
			}
			break;
		case knead_screw_wait_to_complete:
			if((Knead_screw_Stepper_Motor.direction == ANTICLOCKWISE)&&((Knead_screw_Stepper_Motor.steps_counter>=Knead_screw_Stepper_Motor.total_no_of_steps) || (pnpProximityValues.leadscrewBottomPositionLimit == DETECTED)))
			{
				HAL_TIM_Base_Stop_IT(&htim14);
				Knead_screw_Stepper_Motor.rpm_counter=0;
				Knead_screw_Stepper_Motor.steps_counter=0;
				Knead_screw_Stepper_Motor.total_no_of_steps =0;
				errorCntVal.leadscrewErrorCnt = 0;
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
				Knead_movement_to_xx_mm_state=knead_screw_completed;
			}
			else if((Knead_screw_Stepper_Motor.direction == CLOCKWISE) && ((Knead_screw_Stepper_Motor.steps_counter>=Knead_screw_Stepper_Motor.total_no_of_steps) || (pnpProximityValues.leadscrewTopPositionLimit == DETECTED)))
			{
				HAL_TIM_Base_Stop_IT(&htim14);
				Knead_screw_Stepper_Motor.rpm_counter=0;
				Knead_screw_Stepper_Motor.steps_counter=0;
				Knead_screw_Stepper_Motor.total_no_of_steps =0;
				errorCntVal.leadscrewErrorCnt = 0;
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
				Knead_movement_to_xx_mm_state=knead_screw_completed;
			}
			break;
		case knead_screw_completed:
			break;
		}

		switch(knead_screw_rear_end_or_bottom_end_limit_state)
		{
		case knead_screw_idle:
			break;
		case knead_screw_start:
			errorCntVal.leadscrewErrorCnt = 0;
			if(Knead_screw_Stepper_Motor.rpm > 100)
			{
				Knead_screw_Stepper_Motor.rpm_counter = 100;
			}
			if(pnpProximityValues.leadscrewBottomPositionLimit == DETECTED)
			{
				knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_completed;
			}
			else
			{
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, SET);
				HAL_GPIO_WritePin(LEADSCREW_DIR_GPIO_Port,LEADSCREW_DIR_Pin, RESET);
				HAL_TIM_Base_Start_IT(&htim14);
				Knead_screw_Stepper_Motor.direction = CLOCKWISE;
				knead_screw_rear_end_or_bottom_end_limit_state=knead_scerw_acceration;
			}
			break;
		case knead_scerw_acceration:
			Knead_screw_Stepper_Motor.rpm_counter++;
			if(Knead_screw_Stepper_Motor.rpm_counter<Knead_screw_Stepper_Motor.rpm)
			{
				Knead_screw_Stepper_Motor.frequency=(uint32_t)((Knead_screw_Stepper_Motor.rpm_counter*360)/(Knead_screw_Stepper_Motor.step_angle*60))*2;
				Knead_screw_Stepper_Motor.prescaler=(uint32_t)(Knead_screw_Stepper_Motor.timer_clock/(Knead_screw_Stepper_Motor.Period*Knead_screw_Stepper_Motor.frequency));
				TIM14->ARR=(Knead_screw_Stepper_Motor.Period-1);
				TIM14->PSC=(Knead_screw_Stepper_Motor.prescaler-1);
			}
			else
			{
				Knead_screw_Stepper_Motor.rpm_counter=0;
				knead_screw_rear_end_or_bottom_end_limit_state=ejecter_wait_to_complete;
			}
			break;
		case knead_screw_wait_to_complete:
			if(pnpProximityValues.leadscrewBottomPositionLimit == DETECTED)
			{
				HAL_TIM_Base_Stop_IT(&htim14);
				Knead_screw_Stepper_Motor.rpm_counter=0;
				Knead_screw_Stepper_Motor.steps_counter=0;
				Knead_screw_Stepper_Motor.total_no_of_steps =0;
				HAL_GPIO_WritePin(LEADSCREW_EN_GPIO_Port, LEADSCREW_EN_Pin, RESET);
				HAL_GPIO_WritePin(LEADSCREW_PULSE_GPIO_Port, LEADSCREW_PULSE_Pin, RESET);
				errorCntVal.leadscrewErrorCnt = 0;
				kneadLeadscrewTravel.previousMMTravel=kneadLeadscrewTravel.maximumMMTravel;
				knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_completed;
			}
			break;
		case knead_screw_completed:
			break;
		}

		/*Dough Release State-Added on 11-12-2020 to be tested*/
		switch(doughReleaseProcess)
		{
		case doughReleaseIdle:
			break;
		case doughReleaseStart:
			//Update the dough release cycle leadscrew distance
			kneadLeadscrewTravel.newMMTravel = (Dough_Pizza_Eject.Dough_Base_Step_11 /** leadScrewMotor.stepPerMMRotation*/);			//Parameter to be added
			Knead_movement_to_xx_mm_state = knead_screw_start;
			doughReleaseProcess = doughReleaseRounding;
			break;
		case doughReleaseRounding:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				/**Change the structure to store**/
				if(Dough_Pizza_Eject.Dough_release_cycle_clockwise_and_anticlockwise > 0)
				{
					toggleMotorClockAntiClockCnt = Dough_Pizza_Eject.Dough_release_cycle_clockwise_and_anticlockwise;
					Clockwise_on_time = Dough_Pizza_Eject.Dough_release_clockwise_time;
					Anti_Clockwise_on_time = Dough_Pizza_Eject.Dough_releae_anit_clokcwise_time;
					Toggle_off_time_android = Dough_Pizza_Eject.Delay_between_cycles;
					timerCount.togglingTimerCnt = 0;
					Toggle_motor_main_state = Toggle_clockwise_anticlockwise;
					Toggle_motor_clockwise_anticlockwise = Toggle_loop_check;
				}
				else
				{
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				/**Change the structure to store**/
				doughReleaseProcess = doughRelaseRoundingWaitToComplete;
			}
			break;
		case doughRelaseRoundingWaitToComplete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggleMotorClockAntiClockCnt = 0;
				Clockwise_on_time = 0;
				Anti_Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				Toggle_motor_main_state = Toggle_motor_idle;
				Toggle_motor_clockwise_anticlockwise = Toggle_motor_end;
				doughReleaseProcess = doughReleaseComplete;
			}
			break;
		case doughReleaseComplete:
			break;
		}
		/*Dough Release State-Added on 11-12-2020 to be tested*/

		switch(Kneading_motor_state)
		{
		case Kneading_motor_idle:
			break;
		case Kneading_motor_init:
			if(currentBladePWM > previousBladePWM)
			{
				Kneading_motor_state = kneadingAccelerate;
			}
			else if(currentBladePWM < previousBladePWM)
			{
				Kneading_motor_state = kneadingDecellerate;
			}
			break;
		case kneadingAccelerate:
			bladeRunCnt++;
			if(bladeRunCnt >= BLADE_MAX_ACCE_DECCELERATE_CNT)
			{
				bladeDutyCycle++;
				/*Added on 28-01-2021*/
				if(bladeDutyCycle >= 100)
				{
					bladeDutyCycle = 100;
				}
				/*Added on 28-01-2021*/
				if(bladeDutyCycle >= currentBladePWM &&  bladeDutyCycle <= 100)
				{
					previousBladePWM = currentBladePWM;
					Blade_DC_Set_PWM(bladeDutyCycle, ANTICLOCKWISE);
					Kneading_motor_state = Kneading_motor_completed;
				}
				else
				{
					Blade_DC_Set_PWM(bladeDutyCycle, ANTICLOCKWISE);
				}
				bladeRunCnt = 0;
			}
			break;
		case Kneading_motor_wait:
			waitCnt++;
			if(waitCnt >= 6000)
			{
				currentBladePWM = 70;
				bladeRunCnt = 0;
				Kneading_motor_state = Kneading_motor_init;
			}
			break;
		case kneadingDecellerate:
			bladeRunCnt++;
			if(bladeRunCnt >= BLADE_MAX_ACCE_DECCELERATE_CNT)
			{
				bladeDutyCycle--;
				if(bladeDutyCycle <= currentBladePWM && bladeDutyCycle >= 0)
				{
					previousBladePWM = currentBladePWM;
					Blade_DC_Set_PWM(bladeDutyCycle, ANTICLOCKWISE);
					Kneading_motor_state = Kneading_motor_completed;
				}
				else
				{
					Blade_DC_Set_PWM(bladeDutyCycle, ANTICLOCKWISE);
				}
				bladeRunCnt = 0;
			}
			break;
		case Kneading_motor_start:
			break;

		case Kneading_motor_stop:
			break;

		case Kneading_motor_completed:
			break;
		}
		osDelay(2);
	}
  /* USER CODE END Kneading_Motor_task */
}

/* USER CODE BEGIN Header_Adc_task */
/**
 * @brief Function implementing the  thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Adc_task */
void Adc_task(void *argument)
{
  /* USER CODE BEGIN Adc_task */
	Read_ADC2_Channel_state=ADC2_INIT;
	/* Infinite loop */
	for(;;)
	{
		switch(Read_ADC2_Channel_state)
		{
		case  ADC2_IDLE:

			break;
		case ADC2_INIT:
			LPS_struct.LPSRawADCValue=Adc_channel_reading_for_multiple_inputs(hadc2,ADC2_TOTAL_INPUTCHANNELS);
			/* Formula for m = ((y2-y1)/(x2-x1)) */
			ADC_mm.adc_m = ((float)(LPS_CALBRATE_MAX_MM - (float)LPS_CALIBRATE_MIN_MM)/((float)LPS_CALIBRATE_MAX_ADC - (float) LPS_struct.LPSRawADCValue));
			/* Formula for c = ((y2x1-y1x2)/(y2-y1))*/
			ADC_mm.adc_c = ((((float)LPS_CALIBRATE_MAX_ADC * (float)LPS_CALIBRATE_MIN_MM)-((float)LPS_CALBRATE_MAX_MM * (float) LPS_struct.LPSRawADCValue)) / ((float)LPS_CALIBRATE_MAX_ADC - (float) LPS_struct.LPSRawADCValue));
			LPS_struct.calibratedValueinMM=((float)LPS_struct.LPSRawADCValue*(float)ADC_mm.adc_m+(float)(ADC_mm.adc_c));
			Read_ADC2_Channel_state=ADC2_READ_DATA;
			break;

		case ADC2_READ_DATA:
			LPS_struct.LPSRawADCValue=Adc_channel_reading_for_multiple_inputs(hadc2,ADC2_TOTAL_INPUTCHANNELS);
			LPS_struct.calibratedValueinMM=((float)LPS_struct.LPSRawADCValue*(float)ADC_mm.adc_m+(float)(ADC_mm.adc_c));
			if(LPS_struct.calibratedValueinMM >= (((float)(thicknessReferenceValueAndroid - thicknessValueFromAndroid))-PRESS_STOP_OFFSET) && Press_motor_state == press_till_xx_mm_thickness)
			{
				Press_DC_Motor_Stop();
				timerCount.pressTimerCnt = 0;
				Press_motor_state = press_till_xx_mm_thickness_wait;
			}
			break;
		}
		osDelay(1);
	}
  /* USER CODE END Adc_task */
}

/* USER CODE BEGIN Header_Cleaning */
/**
 * @brief Function implementing the Cleaning_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Cleaning */
void Cleaning(void *argument)
{
  /* USER CODE BEGIN Cleaning */
	/* Infinite loop */
	for(;;)
	{
		switch(Cleaning_state_t)
		{
		case cleaning_idle:

			break;
			/*Kneader Lead screw Home(Top) position*/
		case kneader_homing_or_rear_end_init:
			if(pnpProximityValues.ejectorStartPositionLimit == DETECTED)
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_start;
				Cleaning_state_t=kneader_homing_or_rear_end_wait;
			}
			else
			{
				if(pnpProximityValues.pressTopPositionLimit == DETECTED)
				{
					ejecter_home_position_state=ejecter_start;
					Cleaning_state_t=kneader_home_waiting_to_ejecter_home_position;
				}
				else
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
					Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					Press_motor_state=Press_motor_homing_init;
					Cleaning_state_t=kneader_home_waiting_to_Press_home_position;
				}
			}
			break;
		case kneader_home_waiting_to_ejecter_home_position:
			if(pnpProximityValues.ejectorStartPositionLimit == DETECTED && (ejecter_home_position_state==ejecter_completed))
			{
				ejecter_home_position_state=ejecter_idle;
				Cleaning_state_t=kneader_homing_or_rear_end_init;
			}
			break;
		case kneader_home_waiting_to_Press_home_position:
			if(pnpProximityValues.pressTopPositionLimit == DETECTED && Press_motor_state==Press_motor_completed)
			{
				Press_motor_state=Press_motor_idle;
				Cleaning_state_t=kneader_homing_or_rear_end_init;
			}
			break;
		case kneader_homing_or_rear_end_wait:
			if(knead_screw_homing_or_top_end_limit_state==knead_screw_completed)
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
			/*Kneader Lead screw Home(Top) position*/

			/*Kneader Lead screw Bottom position*/
		case	kneader_bottom_end_init:
			if(pnpProximityValues.ejectorStartPositionLimit == DETECTED)
			{
				knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_start;
				Cleaning_state_t=kneader_bottom_end_wait;
			}
			else
			{
				if(pnpProximityValues.pressTopPositionLimit == DETECTED)
				{
					ejecter_home_position_state=ejecter_start;
					Cleaning_state_t=kneader_bottom_waiting_to_ejecter_home_position;
				}
				else
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
					Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					Press_motor_state=Press_motor_homing_init;
					Cleaning_state_t=kneader_bottom_waiting_to_Press_home_position;
				}
			}
			break;
		case kneader_bottom_waiting_to_ejecter_home_position:
			if(pnpProximityValues.ejectorStartPositionLimit == DETECTED && (ejecter_home_position_state==ejecter_completed))
			{
				ejecter_home_position_state=ejecter_idle;
				Cleaning_state_t=kneader_homing_or_rear_end_init;
			}
			break;
		case kneader_bottom_waiting_to_Press_home_position:
			if(pnpProximityValues.pressTopPositionLimit == DETECTED && Press_motor_state==Press_motor_completed)
			{
				Press_motor_state=Press_motor_idle;
				Cleaning_state_t=kneader_homing_or_rear_end_init;
			}
			break;
		case	kneader_bottom_end_wait:
			if(knead_screw_rear_end_or_bottom_end_limit_state==knead_screw_completed)
			{
				knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
			/*Kneader Lead screw Bottom position*/

			/*Kneader Lead screw Cleaning position*/
		case	kneader_Cleaning_position_init:
			if(pnpProximityValues.ejectorStartPositionLimit == DETECTED)
			{
				knead_screw_homing_or_top_end_limit_state = knead_screw_start;
				Cleaning_state_t=kneader_Cleaning_home_position_wait;
			}
			else
			{
				if(pnpProximityValues.pressTopPositionLimit == DETECTED)
				{
					ejecter_home_position_state=ejecter_start;
					Cleaning_state_t=kneader_clean_waiting_to_ejecter_home_position;
				}
				else
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
					Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					Press_motor_state=Press_motor_homing_init;
					Cleaning_state_t=kneader_clean_waiting_to_Press_home_position;
				}
			}
			break;
		case	kneader_clean_waiting_to_ejecter_home_position:
			if(pnpProximityValues.pressTopPositionLimit == DETECTED && ejecter_home_position_state==ejecter_completed)
			{
				ejecter_home_position_state=ejecter_idle;
				knead_screw_homing_or_top_end_limit_state = knead_screw_start;
				Cleaning_state_t=kneader_Cleaning_home_position_wait;
			}
			break;
		case    kneader_clean_waiting_to_Press_home_position:
			if(pnpProximityValues.pressTopPositionLimit == DETECTED && Press_motor_state==Press_motor_completed)
			{
				Press_motor_state=Press_motor_idle;
				knead_screw_homing_or_top_end_limit_state = knead_screw_start;
				Cleaning_state_t=kneader_Cleaning_home_position_wait;
			}
			break;
		case	kneader_Cleaning_home_position_wait:
			if(knead_screw_homing_or_top_end_limit_state==knead_screw_completed)
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_idle;

				//				needd to assign xx_mm value here
				if(commonMCSetting.Kneader_Base_Cleaning_Position > 0)
					kneadLeadscrewTravel.newMMTravel = commonMCSetting.Kneader_Base_Cleaning_Position;
				else
					kneadLeadscrewTravel.newMMTravel = 150;
				Knead_movement_to_xx_mm_state=knead_screw_start;
				Cleaning_state_t=kneader_Cleaning_position_wait;
			}
			break;
		case    kneader_Cleaning_position_wait:
			if(Knead_movement_to_xx_mm_state==knead_screw_completed)
			{
				Knead_movement_to_xx_mm_state=knead_screw_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
			/*Kneader Lead screw Cleaning position*/

			/*Ejector Lead screw Homing(Start) position*/
		case	ejecter_homing_or_rear_end_init:
			if(pnpProximityValues.leadscrewBottomPositionLimit==DETECTED)
			{
				if(pnpProximityValues.pressTopPositionLimit == DETECTED)
				{
					ejecter_home_position_state=ejecter_start;
					Cleaning_state_t=ejecter_homing_or_rear_end_wait;
				}
				else
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
					Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					Press_motor_state=Press_motor_homing_init;
					Cleaning_state_t=ejecter_home_waiting_to_Press_home_position;
				}
			}
			else if(pnpProximityValues.leadscrewTopPositionLimit==DETECTED)
			{
				if(pnpProximityValues.pressTopPositionLimit == DETECTED)
				{
					ejecter_home_position_state=ejecter_start;
					Cleaning_state_t=ejecter_homing_or_rear_end_wait;
				}
				else
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
					Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					Press_motor_state=Press_motor_homing_init;
					Cleaning_state_t=ejecter_home_waiting_to_Press_home_position;
				}
			}
			else
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_start;
				Cleaning_state_t=ejecter_home_waiting_to_kneader_home_position;
			}
			break;
		case	ejecter_home_waiting_to_Press_home_position:
			if(Press_motor_state==Press_motor_completed)
			{
				Press_motor_state=Press_motor_idle;
				Cleaning_state_t=ejecter_homing_or_rear_end_init;
			}
			break;
		case	ejecter_home_waiting_to_kneader_home_position:
			if(knead_screw_homing_or_top_end_limit_state==knead_screw_completed)
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_idle;
				Cleaning_state_t=ejecter_homing_or_rear_end_init;
			}
			break;
		case	ejecter_homing_or_rear_end_wait:
			if(ejecter_home_position_state==ejecter_completed)
			{
				ejecter_home_position_state=ejecter_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
			/*Ejector Lead screw Homing(Start) position*/

			/*Ejector Lead screw Front End(End) position*/
		case	ejecter_front_end_position_init:
			if(pnpProximityValues.leadscrewBottomPositionLimit==DETECTED)
			{
				if(pnpProximityValues.pressTopPositionLimit == DETECTED)
				{
					ejecter_front_end_limit_position_state=ejecter_start;
					Cleaning_state_t=ejecter_front_end_position_wait;
				}
				else
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
					Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					Press_motor_state=Press_motor_homing_init;
					Cleaning_state_t=ejecter_front_waiting_to_Press_home_position;
				}
			}
			else if(pnpProximityValues.leadscrewTopPositionLimit==DETECTED)
			{
				if(pnpProximityValues.pressTopPositionLimit == DETECTED)
				{
					ejecter_front_end_limit_position_state=ejecter_start;
					Cleaning_state_t=ejecter_front_end_position_wait;
				}
				else
				{
					Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
					Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
					Press_motor_state=Press_motor_homing_init;
					Cleaning_state_t=ejecter_front_waiting_to_Press_home_position;
				}
			}
			else
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_start;
				Cleaning_state_t=ejecter_front_waiting_to_kneader_home_position;
			}
			break;
		case	ejecter_front_waiting_to_Press_home_position:
			if(Press_motor_state==Press_motor_completed)
			{
				Press_motor_state=Press_motor_idle;
				Cleaning_state_t=ejecter_front_end_position_init;
			}
			break;
		case	ejecter_front_waiting_to_kneader_home_position:
			if(knead_screw_homing_or_top_end_limit_state==knead_screw_completed)
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_idle;
				Cleaning_state_t=ejecter_front_end_position_init;
			}
			break;
		case	ejecter_front_end_position_wait:
			if(ejecter_front_end_limit_position_state==ejecter_completed)
			{
				ejecter_front_end_limit_position_state=ejecter_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
			/*Ejector Lead screw Front End(End) position*/


		case   water_pump_cleaning_run_init:
			    Water_Stepper_Motor.rpm = HW_Setting1.Water_Pump_Speed;
				Water_Stepper_Motor.direction = CLOCKWISE;
				if(commonMCSetting.Water_cleaning_time > 0)
				{
					waterRunTimeFromAndroid = commonMCSetting.Water_cleaning_time * 1000;
				}
				else
				{
					waterRunTimeFromAndroid = 4000;
				}
//				Water_Stepper_Motor.total_no_of_steps = WATER_PRIMING_REVERSE_STEPS;//incorporate android parameter here
				Water_stepper_maintenance_state = Water_stepper_motor_init;
				Cleaning_state_t = water_pump_cleaning_run_wait;
				break;
		case	water_pump_cleaning_run_wait:
			if(Water_stepper_maintenance_state == Water_stepper_motor_completed)
			{
				timerCount.waterMaintenance = 0;
				Water_stepper_maintenance_state=Water_stepper_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
		case    water_pump_priming_run_init:
		    Water_Stepper_Motor.rpm = HW_Setting1.Water_Pump_Speed;
			Water_Stepper_Motor.direction = ANTICLOCKWISE;
			if(commonMCSetting.Water_priming_time > 0)
			{
				waterRunTimeFromAndroid = commonMCSetting.Water_priming_time * 1000;
			}
			else
			{
				waterRunTimeFromAndroid = 4000;
			}
//			Water_Stepper_Motor.total_no_of_steps = WATER_PRIMING_FORWARD_STEPS;
			Water_stepper_maintenance_state = Water_stepper_motor_init;
			Cleaning_state_t = water_pump_priming_run_wait;
			break;
		case	water_pump_priming_run_wait:
			if(Water_stepper_maintenance_state == Water_stepper_motor_completed)
			{
				timerCount.waterMaintenance = 0;
				Water_stepper_maintenance_state = Water_stepper_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t = cleaning_completed;
			}
			break;
		case   oil_pump_cleaning_run_init:
			Oil_Stepper_Motor.rpm = HW_Setting1.Oil_Pump_Speed;
			Oil_Stepper_Motor.direction = CLOCKWISE;
			if(commonMCSetting.Oil_cleaning_time > 0)
			{
				oilRunTimeFromAndroid = commonMCSetting.Oil_cleaning_time * 1000;
			}
			else
			{
				oilRunTimeFromAndroid = 5000;
			}
			Oil_stepper_maintenance_state = Oil_stepper_motor_init;
			Cleaning_state_t = oil_pump_cleaning_run_wait;
			break;
		case	oil_pump_cleaning_run_wait:
			if(Oil_stepper_maintenance_state == Oil_stepper_motor_completed)
			{
				timerCount.oilMaintenance = 0;
				Oil_stepper_maintenance_state = Oil_stepper_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t = cleaning_completed;
			}
			break;
		case   oil_pump_priming_run_init:
			Oil_Stepper_Motor.rpm = HW_Setting1.Oil_Pump_Speed;
			Oil_Stepper_Motor.direction = ANTICLOCKWISE;
			if(commonMCSetting.Oil_priming_time > 0)
			{
				oilRunTimeFromAndroid = commonMCSetting.Oil_priming_time * 1000;
			}
			else
			{
				oilRunTimeFromAndroid = 5000;
			}
			Oil_stepper_maintenance_state = Oil_stepper_motor_init;
			Cleaning_state_t = oil_pump_priming_run_wait;
			break;
		case	oil_pump_priming_run_wait:
			if(Oil_stepper_maintenance_state == Oil_stepper_motor_completed)
			{
				timerCount.oilMaintenance = 0;
				Oil_stepper_maintenance_state = Oil_stepper_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
		case   flour_motor_cleaning_run_init:
			Flour_Stepper_Motor.rpm = HW_Setting1.Flour_Motor_Speed;
			Flour_Stepper_Motor.direction = ANTICLOCKWISE;
			if(commonMCSetting.Flour_priming_time > 0)
			{
				flourRunTimeFromAndroid = commonMCSetting.Flour_priming_time * 1000;
			}
			else
			{
				flourRunTimeFromAndroid = 4000;
			}
//			Flour_Stepper_Motor.total_no_of_steps = WATER_PRIMING_REVERSE_STEPS;
			Flour_stepper_maintenance_state = Flour_stepper_motor_init;
			Cleaning_state_t = flour_motor_cleaning_run_wait;
			break;
		case	flour_motor_cleaning_run_wait:
			if(Flour_stepper_maintenance_state == Flour_stepper_motor_completed)
			{
				timerCount.flourMaintenance = 0;
				Flour_stepper_maintenance_state = Flour_stepper_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t = cleaning_completed;
			}
			break;
		case   flour_motor_priming_run_init:
			Flour_Stepper_Motor.rpm = HW_Setting1.Flour_Motor_Speed;
			Flour_Stepper_Motor.direction = CLOCKWISE;
			flourRunTimeFromAndroid = commonMCSetting.Flour_priming_time * 1000;
//			Flour_Stepper_Motor.total_no_of_steps = WATER_PRIMING_FORWARD_STEPS;
			Flour_stepper_maintenance_state = Flour_stepper_motor_init;
			Cleaning_state_t = flour_motor_priming_run_wait;
			break;
		case	flour_motor_priming_run_wait:
			if(Flour_stepper_maintenance_state == Flour_stepper_motor_completed)
			{
				Flour_stepper_maintenance_state = Flour_stepper_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t = cleaning_completed;
			}
			break;
		case	press_homing_or_revrse_init:
			if(pnpProximityValues.ejectorStartPositionLimit == DETECTED)
			{
				Pressing_Baking.Press_Motor_reverse_duty_cycle = 40;
				Pressing_Baking.Reverse_Force_Duty_Cycle = Pressing_Baking.Press_Motor_reverse_duty_cycle;
				Press_motor_state=Press_motor_homing_init;
				Cleaning_state_t=press_homing_or_revrse_wait;
			}
			else
			{
				ejecter_home_position_state=ejecter_start;
				Cleaning_state_t=press_home_waiting_to_ejecter_home_position;
			}
			break;
		case	press_home_waiting_to_ejecter_home_position:
			if(ejecter_home_position_state==ejecter_completed)
			{
				Cleaning_state_t=press_homing_or_revrse_init;
				ejecter_home_position_state=ejecter_idle;
			}
			break;
		case	press_homing_or_revrse_wait:
			if(Press_motor_state==Press_motor_completed)
			{
				Press_motor_state=Press_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
		case	press_bottom_limit_or_xxmm_init:
			if(pnpProximityValues.ejectorStartPositionLimit == DETECTED)
			{
				Press_motor_cleaning_bottom = Press_motor_init;
				Cleaning_state_t=press_bottom_limit_or_xxmm_wait;
			}
			else
			{
				ejecter_home_position_state=ejecter_start;
				Cleaning_state_t=press_bottom_waiting_to_ejecter_home_position;
			}
			break;
		case	press_bottom_waiting_to_ejecter_home_position:
			if(ejecter_home_position_state==ejecter_completed)
			{
				Cleaning_state_t=press_bottom_limit_or_xxmm_init;
				ejecter_home_position_state=ejecter_idle;
			}
			break;
		case	press_bottom_limit_or_xxmm_wait:
			if(Press_motor_cleaning_bottom == Press_motor_completed)
			{
				Press_motor_cleaning_bottom = Press_motor_idle;
				Send_ADR_Response(ref_msg,2);
				Cleaning_state_t=cleaning_completed;
			}
			break;
		case cleaning_abort_process:
			 HAL_TIM_Base_Stop_IT(&htim14);
			 HAL_TIM_Base_Stop_IT(&htim7);
			 HAL_TIM_Base_Stop_IT(&htim15);
			 Press_DC_Set_PWM(0, CLOCKWISE);
			 PWM_Channels_Stop();
			 knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_idle;
			 knead_screw_homing_or_top_end_limit_state = knead_screw_idle;
			 Knead_movement_to_xx_mm_state=knead_screw_idle;
			 ejecter_front_end_limit_position_state = ejecter_idle;
			 ejecter_home_position_state = ejecter_idle;
			 Press_motor_cleaning_bottom = Press_motor_idle;
			 Flour_stepper_maintenance_state = Flour_stepper_motor_idle;
			 Water_stepper_maintenance_state = Water_stepper_motor_idle;
			 Oil_stepper_maintenance_state = Oil_stepper_motor_idle;
			 Water_stepper_motor_state = Water_stepper_motor_idle;
			 Oil_stepper_motor_state = Oil_stepper_motor_idle;
			 Flour_stepper_motor_state = Flour_stepper_motor_idle;
			 Press_motor_state=Press_motor_idle;
			 PWM_Channels_Init();
			 Cleaning_state_t=cleaning_idle;
			 break;
		case	cleaning_completed:

			break;
		}
		osDelay(1);
	}
  /* USER CODE END Cleaning */
}

/* USER CODE BEGIN Header_Service_Menu */
/**
 * @brief Function implementing the Service_menu thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Service_Menu */
void Service_Menu(void *argument)
{
  /* USER CODE BEGIN Service_Menu */
	/* Infinite loop */
	for(;;)
	{
		switch(serviceCalibrationState)
		{
			case calibrationIdle:
				break;
			case flourCalibrationStart:
				Flour_Stepper_Motor.rpm = HW_Setting1.Flour_Motor_Speed;
				Flour_Stepper_Motor.direction = CLOCKWISE;
				calibrationFlourRotPerGmFromAndroid = ((float)calibServiceMenuStruct.calibDispenseRotPerGm / (float)10);
				Flour_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml = (STEPS_PER_ROTATION_HALF_STEP  / calibrationFlourRotPerGmFromAndroid);
				Flour_Stepper_Motor.total_no_of_steps = (Flour_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml * (calibServiceMenuStruct.calibDispenseQty));
				Flour_stepper_motor_state = Flour_stepper_motor_init;
				serviceCalibrationState = flourCalibrationWaitToComplete;
				break;
			case flourCalibrationWaitToComplete:
				if(Flour_stepper_motor_state == Flour_stepper_motor_completed)
				{
					Send_ADR_Response(ref_msg, 2);
					Flour_stepper_motor_state = Flour_stepper_motor_idle;
					serviceCalibrationState = serviceCalibrateComplete;
				}
				 break;
			case oilCalibrateStart:
				Oil_Stepper_Motor.rpm = HW_Setting1.Oil_Pump_Speed;
				Oil_Stepper_Motor.direction = ANTICLOCKWISE;
				calibrationOilRotPerGmFromAndroid = ((float)calibServiceMenuStruct.calibDispenseRotPerGm / (float)100);
//				calibrationOilRotPerGmFromAndroid = ((STEPS_PER_ROTATION_HALF_STEP * 100) / (HW_Setting1.Oil_Pump_Flow_Rate));
				Oil_Stepper_Motor.total_no_of_steps = ((STEPS_PER_ROTATION_HALF_STEP / (float)calibrationOilRotPerGmFromAndroid) * ((float)calibServiceMenuStruct.calibDispenseQty ));
				Oil_Stepper_Motor.total_no_of_steps = 400;
				Oil_stepper_motor_state = Oil_stepper_motor_init;
				serviceCalibrationState = oilCalibrateWaitToComplete;
				break;
			case oilCalibrateWaitToComplete:
				if(Oil_stepper_motor_state == Oil_stepper_motor_completed)
				{
					Send_ADR_Response(ref_msg, 2);
					Oil_stepper_motor_state = Oil_stepper_motor_idle;
					serviceCalibrationState = serviceCalibrateComplete;
				}
				break;
			case waterCalibrateStart:
				Water_Stepper_Motor.rpm = HW_Setting1.Water_Pump_Speed;
				Water_Stepper_Motor.direction = ANTICLOCKWISE;
				calibrationWaterRotPerGmFromAndroid = ((float)calibServiceMenuStruct.calibDispenseRotPerGm / (float)10);
				Water_Stepper_Motor.total_no_of_steps = (((STEPS_PER_ROTATION_HALF_STEP)/calibrationWaterRotPerGmFromAndroid) * (calibServiceMenuStruct.calibDispenseQty));
				//Update the number of pulses for motor to rotate to dispense for flow sensor feedback
				water_value = calibServiceMenuStruct.calibDispenseQty * WATER_PULSE_VALUE;
				Water_stepper_motor_state = Water_stepper_motor_init;
				serviceCalibrationState = waterCalibrateWaitToComplete;
				break;
			case waterCalibrateWaitToComplete:
				if(Water_stepper_motor_state == Water_stepper_motor_completed)
				{
					Send_ADR_Response(ref_msg, 2);
					Water_stepper_motor_state = Water_stepper_motor_idle;
					serviceCalibrationState = serviceCalibrateComplete;
				}
				break;
			case serviceCalibrateComplete:
				break;
			case serviceCalibrationAbort:
				 HAL_TIM_Base_Stop_IT(&htim14);
				 HAL_TIM_Base_Stop_IT(&htim7);
				 HAL_TIM_Base_Stop_IT(&htim15);
				 PWM_Channels_Stop();
				 knead_screw_rear_end_or_bottom_end_limit_state=knead_screw_idle;
				 knead_screw_homing_or_top_end_limit_state = knead_screw_idle;
				 Knead_movement_to_xx_mm_state=knead_screw_idle;
				 ejecter_front_end_limit_position_state = ejecter_idle;
				 ejecter_home_position_state = ejecter_idle;
				 Press_motor_cleaning_bottom = Press_motor_idle;
				 Flour_stepper_motor_state = Flour_stepper_motor_idle;
				 Water_stepper_motor_state = Water_stepper_motor_idle;
				 Oil_stepper_motor_state = Oil_stepper_motor_idle;
				 Press_motor_state=Press_motor_idle;
				 PWM_Channels_Init();
				break;
		}
		osDelay(1);
	}
  /* USER CODE END Service_Menu */
}

/* USER CODE BEGIN Header_EjectorTask */
/**
* @brief Function implementing the Ejector_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EjectorTask */
void EjectorTask(void *argument)
{
  /* USER CODE BEGIN EjectorTask */

	Ejecter_Stepper_Motor.rpm=STEPPER_MOTOR_DEFAULT_RPM;
	Ejecter_Stepper_Motor.timer_clock=STEPPER_TIMER_CLOCK;
	Ejecter_Stepper_Motor.step_angle=STEPPER_STEP;
	Ejecter_Stepper_Motor.Period=STEPPER_TIMER_PERIOD;

	Ejecter_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml= EJECTOR_LEADSCREW_STEPS_PER_MM;//66;//200;		//Modified on 16-01-2021

  /* Infinite loop */
  for(;;)
  {
		if(ejecter_front_end_limit_position_state == ejecter_wait_to_complete)
		{
			errorCntVal.ejectorErrorCnt++;
			if(errorCntVal.ejectorErrorCnt >= EJECTOR_ERROR_TIME)
			{
				processErrorState.ejectorLeadscrewEnd = 1;
//				HAL_TIM_Base_Stop_IT(&htim15);
//				ejecter_front_end_limit_position_state = ejecter_idle;
//				ejecter_home_position_state = ejecter_idle;
//				Ejecter_Stepper_Motor.rpm_counter=0;
//				Ejecter_Stepper_Motor.steps_counter=0;
//				Ejecter_Stepper_Motor.total_no_of_steps =0;
//	#if				!MACHINE_1
//				HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
//				HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
//	#elif			MACHINE_1
//				HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
//				HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
//	#endif
			}
		}
		else if( ejecter_home_position_state == ejecter_wait_to_complete)
		{
			errorCntVal.ejectorErrorCnt++;
			if(errorCntVal.ejectorErrorCnt >= EJECTOR_ERROR_TIME)
			{
				processErrorState.ejectorLeadscrewStart = 1;
//				HAL_TIM_Base_Stop_IT(&htim15);
//				ejecter_front_end_limit_position_state = ejecter_idle;
//				ejecter_home_position_state = ejecter_idle;
//				Ejecter_Stepper_Motor.rpm_counter=0;
//				Ejecter_Stepper_Motor.steps_counter=0;
//				Ejecter_Stepper_Motor.total_no_of_steps =0;
//	#if				!MACHINE_1
//				HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
//				HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
//	#elif			MACHINE_1
//				HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
//				HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
//	#endif
			}
		}
	switch(ejecter_front_end_limit_position_state)
	{
	case ejecter_idle:

		break;
	case ejecter_start:
		errorCntVal.ejectorErrorCnt = 0;
#if			!MACHINE_1
		HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, SET);
		HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin,SET);
#elif		MACHINE_1
		HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, SET);
		HAL_GPIO_WritePin(SPARE_STEP_MTR_DIR_GPIO_Port, SPARE_STEP_MTR_DIR_Pin,SET);
#endif
		Ejecter_Stepper_Motor.direction = CLOCKWISE;
		if(Ejecter_Stepper_Motor.rpm_counter > 100)
		{
			Ejecter_Stepper_Motor.rpm_counter = 100;
		}
		HAL_TIM_Base_Start_IT(&htim15);
		ejecter_front_end_limit_position_state=ejecter_acceleration;
		break;
	case ejecter_acceleration:
		Ejecter_Stepper_Motor.rpm_counter++;
		if(Ejecter_Stepper_Motor.rpm_counter<Ejecter_Stepper_Motor.rpm)
		{
			Ejecter_Stepper_Motor.frequency=(uint32_t)((Ejecter_Stepper_Motor.rpm_counter*360)/(Ejecter_Stepper_Motor.step_angle*60))*2;
			Ejecter_Stepper_Motor.prescaler=(uint32_t)(Ejecter_Stepper_Motor.timer_clock/(Ejecter_Stepper_Motor.Period*Ejecter_Stepper_Motor.frequency));
			TIM15->ARR=(Ejecter_Stepper_Motor.Period-1);
			TIM15->PSC=(Ejecter_Stepper_Motor.prescaler-1);
		}
		else
		{
			Ejecter_Stepper_Motor.rpm_counter=0;
			ejecter_front_end_limit_position_state=ejecter_wait_to_complete;
		}
		if(pnpProximityValues.ejectorFrontEndPositionLimit==DETECTED)
		{
			HAL_TIM_Base_Stop_IT(&htim15);
			Ejecter_Stepper_Motor.rpm_counter=0;
			Ejecter_Stepper_Motor.steps_counter=0;
			Ejecter_Stepper_Motor.total_no_of_steps =0;
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
			HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
			ejecter_front_end_limit_position_state=ejecter_completed;
		}
		break;
	case ejecter_wait_to_complete:
		if(pnpProximityValues.ejectorFrontEndPositionLimit==DETECTED)
		{
			HAL_TIM_Base_Stop_IT(&htim15);
			Ejecter_Stepper_Motor.rpm_counter=0;
			Ejecter_Stepper_Motor.steps_counter=0;
			Ejecter_Stepper_Motor.total_no_of_steps =0;
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
			HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
			ejecter_front_end_limit_position_state=ejecter_completed;
			ejectorLeadscrewTravel.previousMMTravel=ejectorLeadscrewTravel.maximumMMTravel;
		}
		break;
	case ejecter_completed:
		errorCntVal.ejectorErrorCnt = 0;
		HAL_TIM_Base_Stop_IT(&htim15);
		Ejecter_Stepper_Motor.rpm_counter=0;
		Ejecter_Stepper_Motor.steps_counter=0;
		Ejecter_Stepper_Motor.total_no_of_steps =0;
#if			!MACHINE_1
		HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
		HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif		MACHINE_1
		HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
		HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
		ejectorLeadscrewTravel.previousMMTravel=ejectorLeadscrewTravel.maximumMMTravel;
		break;
	}


	switch(ejecter_movment_to_xx_mm_state)
	{
	case ejecter_idle:

		break;
	case ejecter_start:
		errorCntVal.ejectorErrorCnt = 0;
		if(Ejecter_Stepper_Motor.rpm_counter > 100)
		{
			Ejecter_Stepper_Motor.rpm_counter = 100;
		}
		if((ejectorLeadscrewTravel.newMMTravel>ejectorLeadscrewTravel.previousMMTravel)&&(ejectorLeadscrewTravel.newMMTravel<=ejectorLeadscrewTravel.maximumMMTravel))
		{
			Ejecter_Stepper_Motor.direction = CLOCKWISE;
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, SET);
			HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin,SET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, SET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_DIR_GPIO_Port, SPARE_STEP_MTR_DIR_Pin,SET);
#endif
			Ejecter_Stepper_Motor.total_no_of_steps = ejectorLeadscrewTravel.newMMTravel-ejectorLeadscrewTravel.previousMMTravel;
			Ejecter_Stepper_Motor.total_no_of_steps = (Ejecter_Stepper_Motor.total_no_of_steps * Ejecter_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml);//becuase 1 cycle will take 2 toggle
			HAL_TIM_Base_Start_IT(&htim15);
			ejectorLeadscrewTravel.previousMMTravel=ejectorLeadscrewTravel.newMMTravel;
			ejecter_movment_to_xx_mm_state=ejecter_acceleration;
		}
		else if((ejectorLeadscrewTravel.newMMTravel<ejectorLeadscrewTravel.previousMMTravel)&&(ejectorLeadscrewTravel.newMMTravel<=ejectorLeadscrewTravel.maximumMMTravel))
		{
			Ejecter_Stepper_Motor.direction = ANTICLOCKWISE;
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin, SET);
			HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin,RESET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, SET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_DIR_GPIO_Port, SPARE_STEP_MTR_DIR_Pin,RESET);
#endif
			Ejecter_Stepper_Motor.total_no_of_steps = ejectorLeadscrewTravel.previousMMTravel-ejectorLeadscrewTravel.newMMTravel;
			Ejecter_Stepper_Motor.total_no_of_steps = (Ejecter_Stepper_Motor.total_no_of_steps * Ejecter_Stepper_Motor.no_of_steps_per_gm_or_mm_or_ml);//becuase 1 cycle will take 2 toggole
			HAL_TIM_Base_Start_IT(&htim15);
			ejectorLeadscrewTravel.previousMMTravel=ejectorLeadscrewTravel.newMMTravel;
			ejecter_movment_to_xx_mm_state=ejecter_acceleration;
		}
		else
		{
			ejecter_movment_to_xx_mm_state=ejecter_completed;
		}
		break;
	case ejecter_acceleration:
		Ejecter_Stepper_Motor.rpm_counter++;
		if(Ejecter_Stepper_Motor.rpm_counter<Ejecter_Stepper_Motor.rpm)
		{
			Ejecter_Stepper_Motor.frequency=(uint32_t)((Ejecter_Stepper_Motor.rpm_counter*360)/(Ejecter_Stepper_Motor.step_angle*60))*2;
			Ejecter_Stepper_Motor.prescaler=(uint32_t)(Ejecter_Stepper_Motor.timer_clock/(Ejecter_Stepper_Motor.Period*Ejecter_Stepper_Motor.frequency));
			TIM15->ARR=(Ejecter_Stepper_Motor.Period-1);
			TIM15->PSC=(Ejecter_Stepper_Motor.prescaler-1);
		}
		else
		{
			Ejecter_Stepper_Motor.rpm_counter=0;
			ejecter_movment_to_xx_mm_state=ejecter_wait_to_complete;
		}
		if((pnpProximityValues.ejectorFrontEndPositionLimit==DETECTED || Ejecter_Stepper_Motor.steps_counter>=Ejecter_Stepper_Motor.total_no_of_steps)
				&& Ejecter_Stepper_Motor.direction == CLOCKWISE )
		{
			HAL_TIM_Base_Stop_IT(&htim15);
			Ejecter_Stepper_Motor.rpm_counter=0;
			Ejecter_Stepper_Motor.steps_counter=0;
			Ejecter_Stepper_Motor.total_no_of_steps =0;
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
			HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
			ejecter_movment_to_xx_mm_state=ejecter_completed;
		}
		else if((pnpProximityValues.ejectorStartPositionLimit == DETECTED || Ejecter_Stepper_Motor.steps_counter>=Ejecter_Stepper_Motor.total_no_of_steps)
				&& Ejecter_Stepper_Motor.direction == ANTICLOCKWISE)
		{
			HAL_TIM_Base_Stop_IT(&htim15);
			Ejecter_Stepper_Motor.rpm_counter=0;
			Ejecter_Stepper_Motor.steps_counter=0;
			Ejecter_Stepper_Motor.total_no_of_steps =0;
#if			!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
			HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif		MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
			ejecter_movment_to_xx_mm_state=ejecter_completed;
		}
		break;
	case ejecter_wait_to_complete:
		if(((pnpProximityValues.ejectorFrontEndPositionLimit==DETECTED)
				||( Ejecter_Stepper_Motor.steps_counter>=Ejecter_Stepper_Motor.total_no_of_steps))
				&&(Ejecter_Stepper_Motor.direction == CLOCKWISE))
		{
			HAL_TIM_Base_Stop_IT(&htim15);
			Ejecter_Stepper_Motor.rpm_counter=0;
			Ejecter_Stepper_Motor.steps_counter=0;
			Ejecter_Stepper_Motor.total_no_of_steps =0;
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
			HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
			ejecter_movment_to_xx_mm_state=ejecter_completed;
		}
		else if(((pnpProximityValues.ejectorStartPositionLimit==DETECTED)
				||( Ejecter_Stepper_Motor.steps_counter>=Ejecter_Stepper_Motor.total_no_of_steps))
				&&(Ejecter_Stepper_Motor.direction==ANTICLOCKWISE))
		{
			HAL_TIM_Base_Stop_IT(&htim15);
			Ejecter_Stepper_Motor.rpm_counter=0;
			Ejecter_Stepper_Motor.steps_counter=0;
			Ejecter_Stepper_Motor.total_no_of_steps =0;
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
			HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
			ejecter_movment_to_xx_mm_state=ejecter_completed;
		}
		break;
	case ejecter_completed:
		errorCntVal.ejectorErrorCnt = 0;
		HAL_TIM_Base_Stop_IT(&htim15);
		Ejecter_Stepper_Motor.rpm_counter=0;
		Ejecter_Stepper_Motor.steps_counter=0;
		Ejecter_Stepper_Motor.total_no_of_steps =0;
#if			!MACHINE_1
		HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
		HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif		MACHINE_1
		HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
		HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
		break;
	}

	//		pnpProximityValues.ejectorFrontEndPositionLimit=HAL_GPIO_ReadPin(EJECTOR_END_POS_GPIO_Port, EJECTOR_END_POS_Pin);
	//		pnpProximityValues.ejectorStartPositionLimit=HAL_GPIO_ReadPin(EJECTOR_START_POS_GPIO_Port, EJECTOR_START_POS_Pin);
	pnpProximityValues.ejectorMidPositionLimit=HAL_GPIO_ReadPin(EJECTOR_MID_POS_GPIO_Port, EJECTOR_MID_POS_Pin);

	switch(ejecter_home_position_state)
	{
	case ejecter_idle:

		break;
	case ejecter_start:
		errorCntVal.ejectorErrorCnt = 0;
		if(pnpProximityValues.ejectorStartPositionLimit == DETECTED)
		{
			ejecter_home_position_state=ejecter_completed;
		}
		else
		{
#if				!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_DIR_GPIO_Port, EJECTOR_DIR_Pin, RESET);
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,SET);
#elif			MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, SET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_DIR_GPIO_Port, SPARE_STEP_MTR_DIR_Pin,RESET);
#endif
			if(Ejecter_Stepper_Motor.rpm_counter > 100)
			{
				Ejecter_Stepper_Motor.rpm_counter = 100;
			}
			HAL_TIM_Base_Start_IT(&htim15);
			Ejecter_Stepper_Motor.direction = ANTICLOCKWISE;
			ejecter_home_position_state=ejecter_acceleration;
		}
		break;
	case ejecter_acceleration:
		Ejecter_Stepper_Motor.rpm_counter++;
		if(Ejecter_Stepper_Motor.rpm_counter<Ejecter_Stepper_Motor.rpm)
		{
			Ejecter_Stepper_Motor.frequency=(uint32_t)((Ejecter_Stepper_Motor.rpm_counter*360)/(Ejecter_Stepper_Motor.step_angle*60))*2;
			Ejecter_Stepper_Motor.prescaler=(uint32_t)(Ejecter_Stepper_Motor.timer_clock/(Ejecter_Stepper_Motor.Period*Ejecter_Stepper_Motor.frequency));
			TIM15->ARR=(Ejecter_Stepper_Motor.Period-1);
			TIM15->PSC=(Ejecter_Stepper_Motor.prescaler-1);
		}
		else
		{
			Ejecter_Stepper_Motor.rpm_counter=0;
			ejecter_home_position_state=ejecter_wait_to_complete;
		}
		break;
	case ejecter_wait_to_complete:
		if(pnpProximityValues.ejectorStartPositionLimit == DETECTED)
		{
			HAL_TIM_Base_Stop_IT(&htim15);
			Ejecter_Stepper_Motor.rpm_counter=0;
			Ejecter_Stepper_Motor.steps_counter=0;
			Ejecter_Stepper_Motor.total_no_of_steps =0;
#if			!MACHINE_1
			HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
			HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif 		MACHINE_1
			HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
			HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
			ejectorLeadscrewTravel.previousMMTravel=0;
			ejecter_home_position_state=ejecter_completed;
		}
		break;
	case ejecter_completed:
		errorCntVal.ejectorErrorCnt = 0;
		HAL_TIM_Base_Stop_IT(&htim15);
		Ejecter_Stepper_Motor.rpm_counter=0;
		Ejecter_Stepper_Motor.steps_counter=0;
		Ejecter_Stepper_Motor.total_no_of_steps =0;
#if			!MACHINE_1
		HAL_GPIO_WritePin(EJECTOR_EN_GPIO_Port, EJECTOR_EN_Pin,RESET);
		HAL_GPIO_WritePin(EJECTOR_PULSE_GPIO_Port, EJECTOR_PULSE_Pin,RESET);
#elif		MACHINE_1
		HAL_GPIO_WritePin(SPARE_STEP_MTR_EN_GPIO_Port, SPARE_STEP_MTR_EN_Pin, RESET);
		HAL_GPIO_WritePin(SPARE_STEP_MTR_PULSE_GPIO_Port, SPARE_STEP_MTR_PULSE_Pin,RESET);
#endif
		ejectorLeadscrewTravel.previousMMTravel=0;
		break;
	}
    osDelay(1);
  }
  /* USER CODE END EjectorTask */
}

/* USER CODE BEGIN Header_TogglingTask */
/**
* @brief Function implementing the toggling_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TogglingTask */
void TogglingTask(void *argument)
{
  /* USER CODE BEGIN TogglingTask */
  /* Infinite loop */
  for(;;)
  {
    switch(Toggle_motor_main_state)
	{
	case Toggle_motor_completed:
		break;
	case Toggle_motor_idle:
		break;
	case Toggle_clockwise:
		/*CLOCKWISE TOGGLE CASE*/
		switch(Toggle_motor_clockwise)
		{
			case Toggle_loop_check:
				if((toggle_motor_clockwise_counts > 0 && toggle_count < toggle_motor_clockwise_counts))
				{
					Toggle_motor_clockwise = Toggle_motor_acceleration_clockwise;
				}
				else if(toggle_count >= toggle_motor_clockwise_counts)
				{
					toggle_count = 0;
					Toggle_motor_clockwise = Toggle_motor_end;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				break;
			case Toggle_motor_acceleration_clockwise:
				toggleCnt++;
				if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
				{
					togglePWMDutyCycle += 1;
					/*Added on 28-01-2021*/
					if(togglePWMDutyCycle >= 100)
					{
						togglePWMDutyCycle = 100;
					}
					/*Added on 28-01-2021*/
					if(togglePWMDutyCycle >= HW_Setting1.Rounding_Motor_Speed &&  togglePWMDutyCycle <= 100)
					{
						Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
						timerCount.togglingTimerCnt = 0;
						Toggle_motor_clockwise=Toggle_motor_clockwise_on_wait;
					}
					else if(togglePWMDutyCycle < 100)
					{
						Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
					}
					toggleCnt = 0;
				}
				break;
			case Toggle_motor_clockwise_on:
				Toggling_DC_Set_PWM(85.0,CLOCKWISE);
				timerCount.togglingTimerCnt = 0;
				Toggle_motor_clockwise=Toggle_motor_clockwise_on_wait;
				break;
			case Toggle_motor_clockwise_on_wait:
				if(timerCount.togglingTimerCnt >= Clockwise_on_time)
				{
					Clockwise_on_time = 0;
					timerCount.togglingTimerCnt=0;
					Toggle_motor_clockwise = Toggle_motor_decceleration_clockwise;
				}
				break;
			case Toggle_motor_decceleration_clockwise:
				toggleCnt++;
				if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
				{
					togglePWMDutyCycle -= 1;
					if(togglePWMDutyCycle <= 0)
					{
						Toggling_DC_Motor_Stop();
						togglePWMDutyCycle=0;
	//						Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
						timerCount.togglingTimerCnt = 0;
						Toggle_motor_clockwise=Toggle_motor_clockwise_offtime_wait;
					}
					else
					{
						Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
					}
					toggleCnt = 0;
				}
				break;
			case Toggle_motor_clockwise_offtime_wait:
				if(Toggle_off_time_android == 0)
				{
					timerCount.togglingTimerCnt = 0;
					togglePWMDutyCycle = 0;
					toggle_count = 0;
					Toggle_motor_clockwise = Toggle_motor_end;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				if(timerCount.togglingTimerCnt >= Toggle_off_time_android && (toggle_motor_clockwise_counts > 0 && toggle_count < toggle_motor_clockwise_counts))
				{
					Toggle_off_time_android = 0;
					timerCount.togglingTimerCnt = 0;
					togglePWMDutyCycle = 0;
					toggle_count++;
					Toggle_motor_clockwise = Toggle_loop_check;
				}
				else if(timerCount.togglingTimerCnt >= Toggle_off_time_android && (toggle_motor_clockwise_counts > 0 && toggle_count >= toggle_motor_clockwise_counts ))
				{
					Toggle_off_time_android = 0;
					timerCount.togglingTimerCnt = 0;
					togglePWMDutyCycle = 0;
					toggle_count = 0;
					Toggle_motor_clockwise = Toggle_motor_end;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				break;
			case Toggle_motor_end:
				break;
		}
		/*CLOCKWISE TOGGLE CASE*/
		break;
		/*ANTI-CLOCKWISE TOGGLE CASE*/
		case Toggle_anticlockwise:
			switch(Toggle_motor_anticlockwise)
			{
			case Toggle_loop_check:
				if(Coating_Rounding.Rouding_Clock_wise_cycles > 0 && toggle_count < Coating_Rounding.Rouding_Clock_wise_cycles)
				{
					Toggle_motor_anticlockwise = Toggle_motor_acceleration_anti_clockwise;
				}
				else if(toggle_count >= Coating_Rounding.Rouding_Clock_wise_cycles)
				{
					toggle_count = 0;
					Toggle_motor_anticlockwise = Toggle_motor_end;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				break;
			case Toggle_motor_acceleration_anti_clockwise:
				toggleCnt++;
				if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
				{
					togglePWMDutyCycle++;
					if(togglePWMDutyCycle >= HW_Setting1.Rounding_Motor_Speed &&  togglePWMDutyCycle <= 100)
					{
						Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
						timerCount.togglingTimerCnt = 0;
						Toggle_motor_clockwise=Toggle_motor_anti_clockwise_on_wait;
					}
					else
					{
						Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
					}
					toggleCnt = 0;
				}
				break;
			case Toggle_motor_anti_clockwise_on:
				Toggling_DC_Set_PWM(85.0, ANTICLOCKWISE);
				Toggle_motor_anticlockwise=Toggle_motor_anti_clockwise_on_wait;
				break;
			case Toggle_motor_anti_clockwise_on_wait:
				if(timerCount.togglingTimerCnt >= Coating_Rounding.Rounding_time)
				{
					//						Toggling_DC_Motor_Stop();
					Toggle_motor_anticlockwise=Toggle_motor_decceleration_anti_clockwise;
					timerCount.togglingTimerCnt=0;
				}
				break;
			case Toggle_motor_decceleration_anti_clockwise:
				toggleCnt++;
				if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
				{
					togglePWMDutyCycle--;
					if(togglePWMDutyCycle <= 0)
					{
						Toggling_DC_Motor_Stop();
						//							 Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
						timerCount.togglingTimerCnt = 0;
						Toggle_motor_clockwise=Toggle_motor_clockwise_offtime_wait;
					}
					else
					{
						Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
					}
					toggleCnt = 0;
				}
				break;
			case Toggle_motor_clockwise_offtime_wait:
				if(timerCount.togglingTimerCnt >= Coating_Rounding.delay_between_cycles && Coating_Rounding.Rouding_Clock_wise_cycles > 0 && toggle_count < Coating_Rounding.Rouding_Clock_wise_cycles )
				{
					togglePWMDutyCycle = 0;
					Toggling_DC_Motor_Stop();
					toggle_count++;
					timerCount.togglingTimerCnt = 0;
					Toggle_motor_anticlockwise = Toggle_loop_check;
				}
				else if(timerCount.togglingTimerCnt >= Coating_Rounding.delay_between_cycles && Coating_Rounding.Rouding_Clock_wise_cycles > 0 && toggle_count >= Coating_Rounding.Rouding_Clock_wise_cycles )
				{
					togglePWMDutyCycle = 0;
					Toggling_DC_Motor_Stop();
					toggle_count = 0;
					timerCount.togglingTimerCnt = 0;
					Toggle_motor_anticlockwise = Toggle_motor_end;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				break;
			case Toggle_motor_end:
				break;
			}
			/*ANTI-CLOCKWISE TOGGLE CASE*/
			break;
			/*CLOCKWISE & ANTI-CLOCKWISE TOGGLE CASE*/
			// toggleMotorClockAntiClockCnt = Coating_Rounding.Roudnig_motor_clockwise_and_anitclockwise_cycles
			case Toggle_clockwise_anticlockwise:
				switch(Toggle_motor_clockwise_anticlockwise)
				{
				case Toggle_loop_check:
					if(toggleMotorClockAntiClockCnt > 0 && toggle_count < toggleMotorClockAntiClockCnt)
					{
						togglePWMDutyCycle = 0;		//Added on 20-Feb
						timerCount.togglingTimerCnt = 0;	//Added on 20-Feb
						Toggle_motor_clockwise_anticlockwise = Toggle_motor_acceleration_clockwise;
					}
					else if(toggle_count >= toggleMotorClockAntiClockCnt)
					{
						toggle_count = 0;
						Toggle_motor_clockwise_anticlockwise = Toggle_motor_end;
						Toggle_motor_main_state = Toggle_motor_completed;
					}
					break;
				case Toggle_motor_acceleration_clockwise:
					toggleCnt++;
					if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
					{
						togglePWMDutyCycle += 1;
						/*Added on 28-01-2021*/
						if(togglePWMDutyCycle >= 100)
						{
							togglePWMDutyCycle = 100;
						}
						/*Added on 28-01-2021*/
						if(togglePWMDutyCycle >= HW_Setting1.Rounding_Motor_Speed &&  togglePWMDutyCycle <= 100)
						{
							Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
							timerCount.togglingTimerCnt = 0;
							Toggle_motor_clockwise_anticlockwise=Toggle_motor_clockwise_on_wait;
						}
						else if(togglePWMDutyCycle < 100)
						{
							Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
						}
						toggleCnt = 0;
					}
					break;
				case Toggle_motor_clockwise_on_wait:
					if(timerCount.togglingTimerCnt >= Clockwise_on_time)
					{
//							Clockwise_on_time = 0;
						//							Toggling_DC_Motor_Stop();
						Toggle_motor_clockwise_anticlockwise=Toggle_motor_decceleration_clockwise;
						timerCount.togglingTimerCnt=0;
					}
					break;
				case Toggle_motor_decceleration_clockwise:
					toggleCnt++;
					if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
					{
						togglePWMDutyCycle -= 1;
						if(togglePWMDutyCycle <= 0)
						{
							togglePWMDutyCycle = 0;
							Toggling_DC_Motor_Stop();
							//								 Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
//								timerCount.togglingTimerCnt = 0;
							Toggle_motor_clockwise_anticlockwise=Toggle_motor_clockwise_offtime_wait;
						}
						else if(togglePWMDutyCycle > 0 && togglePWMDutyCycle < 100)
						{
							Toggling_DC_Set_PWM(togglePWMDutyCycle, CLOCKWISE);
						}
						toggleCnt = 0;
					}
					break;
				case Toggle_motor_clockwise_offtime_wait:
					if(timerCount.togglingTimerCnt >= Toggle_off_time_android && toggleMotorClockAntiClockCnt > 0 && toggle_count < toggleMotorClockAntiClockCnt )
					{
						Toggling_DC_Motor_Stop();
						timerCount.togglingTimerCnt = 0;
						Toggle_motor_clockwise_anticlockwise = Toggle_motor_acceleration_anti_clockwise;
					}
					break;
				case Toggle_motor_acceleration_anti_clockwise:
					toggleCnt++;
					if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
					{
						togglePWMDutyCycle += 1;
						/*Added on 28-01-2021*/
						if(togglePWMDutyCycle >= 100)
						{
							togglePWMDutyCycle = 100;
						}
						/*Added on 28-01-2021*/
						if(togglePWMDutyCycle >= HW_Setting1.Rounding_Motor_Speed &&  togglePWMDutyCycle <= 100)
						{
							Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
							 timerCount.togglingTimerCnt = 0;
							Toggle_motor_clockwise_anticlockwise=Toggle_motor_anti_clockwise_on_wait;
						}
						else if(togglePWMDutyCycle < 100)
						{
							Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
						}
						toggleCnt = 0;
					}
					break;
				case Toggle_motor_anti_clockwise_on:
					Toggling_DC_Set_PWM(95.0, ANTICLOCKWISE);
					timerCount.togglingTimerCnt = 0;
					Toggle_motor_clockwise_anticlockwise=Toggle_motor_anti_clockwise_on_wait;
					break;
				case Toggle_motor_anti_clockwise_on_wait:
					if(timerCount.togglingTimerCnt >= Anti_Clockwise_on_time)
					{
//						Toggling_DC_Motor_Stop();
						Toggle_motor_clockwise_anticlockwise=Toggle_motor_decceleration_anti_clockwise;
						timerCount.togglingTimerCnt=0;
					}
					break;
				case Toggle_motor_decceleration_anti_clockwise:
					toggleCnt++;
					if(toggleCnt >= TOGGLE_MAX_ACCE_DECCELERATE_CNT)
					{
						togglePWMDutyCycle--;
						if(togglePWMDutyCycle <= 0)
						{
							Toggling_DC_Motor_Stop();
							//								 Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
							timerCount.togglingTimerCnt = 0;
							Toggle_motor_clockwise_anticlockwise=Toggle_motor_anticlockwise_offtime_wait;
						}
						else if(togglePWMDutyCycle < 100 && togglePWMDutyCycle > 0)
						{
							Toggling_DC_Set_PWM(togglePWMDutyCycle, ANTICLOCKWISE);
						}
						toggleCnt = 0;
					}
					break;
				case Toggle_motor_anticlockwise_offtime_wait:
					if(timerCount.togglingTimerCnt >= Toggle_off_time_android && toggleMotorClockAntiClockCnt > 0 && toggle_count < toggleMotorClockAntiClockCnt )
					{
						togglePWMDutyCycle = 0;
						Toggling_DC_Motor_Stop();
						toggle_count++;
						timerCount.togglingTimerCnt = 0;
						Toggle_motor_clockwise_anticlockwise = Toggle_loop_check;
					}
					else if(timerCount.togglingTimerCnt >= Toggle_off_time && toggleMotorClockAntiClockCnt > 0 && toggle_count >= toggleMotorClockAntiClockCnt )
					{
						togglePWMDutyCycle = 0;
						Toggling_DC_Motor_Stop();
						toggle_count = 0;
						timerCount.togglingTimerCnt = 0;
						Toggle_motor_clockwise_anticlockwise = Toggle_motor_end;
						Toggle_motor_main_state = Toggle_motor_completed;
					}
					break;
				case Toggle_motor_end:
					Toggling_DC_Motor_Stop();
					break;
				}
				/*CLOCKWISE & ANTI-CLOCKWISE TOGGLE CASE*/
				break;
	}
    osDelay(2);
  }
  /* USER CODE END TogglingTask */
}

/* USER CODE BEGIN Header_Kneading_Flow_Task */
/**
* @brief Function implementing the kneading_Proces thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Kneading_Flow_Task */
void Kneading_Flow_Task(void *argument)
{
  /* USER CODE BEGIN Kneading_Flow_Task */
  /* Infinite loop */
  for(;;)
  {
		switch(kneadingProcessState)
		{
		case kneading_idle:
			break;
		case kneading_top_proximity_check_init:
			//Check for top proximity sensor for kneading process to start
			if(Pizza_setting.Dough_shape == PIZZA_BASE_MODE) 		//Base mode
			{
				dough_ejection_complete = 0;		//Made 0 for multiple pizza to be done continuously
			}
			if(pnpProximityValues.leadscrewTopPositionLimit==DETECTED)
			{
				kneadLeadscrewTravel.previousMMTravel=0;
				kneadingProcessState=kneading_start_process;
			}
			else
			{
				knead_screw_homing_or_top_end_limit_state=knead_screw_start;
				kneadingProcessState=kneading_top_proximity_check;
			}
			break;
		case kneading_top_proximity_check:
			//Check for Top proximity state for completion
			if(knead_screw_homing_or_top_end_limit_state==knead_screw_completed && pnpProximityValues.leadscrewTopPositionLimit==DETECTED)
			{
				kneadLeadscrewTravel.previousMMTravel=0;
				knead_screw_homing_or_top_end_limit_state=knead_screw_idle;
				kneadingProcessState=kneading_start_process;
			}
			break;
		case kneading_start_process:
			//Move the base down for kneading to have gap between the blade and jar
			bladeDutyCycle = 0;
			kneading_process_percent = 0;
			kneadLeadscrewTravel.newMMTravel = (commonMCSetting.Gap_between_blade_and_dough_base);			//Distance b/w blade and base of dough
			leadscrewMotorKneadingDirection = CLOCKWISE;
			Knead_movement_to_xx_mm_state = knead_screw_start;
			kneadingProcessState = kneading_leadscrew_down_for_blade_and_jar_gap;
			break;
		case kneading_leadscrew_down_for_blade_and_jar_gap:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneadingProcessState = kneading_start_blade_motor_at_first_rpm;
			}
			break;
		case kneading_start_blade_motor_at_first_rpm:
			//Start the DC Blade motor
#if 		bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Ingredient_mixing_parameters.Mixing_speed_1;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;
				Kneading_motor_state = Kneading_motor_init;
			}
#elif		!bladePWM
			HAL_GPIO_WritePin(DC_SSR_OP1_GPIO_Port, DC_SSR_OP1_Pin, SET);
#endif
			kneading_process_percent = 5;
			kneadingProcessState = kneading_dispense_flour_qty1;
			break;
		case kneading_dispense_flour_qty1:
			//Flour dispense quantity
			//Update the number of steps for motor to rotate to dispense
#if			!KNEADING_TIME_REDUCE
			if(Kneading_motor_state == Kneading_motor_completed)
#endif
#if 		FLOUR_PRESENCE_SENSOR
			if(pnpProximityValues.flourConnectorPresence == 1)
#endif
			{
				Flour_Stepper_Motor.total_no_of_steps =Ingredient_mixing_parameters.Flour_Qty1* HW_Setting1.Flour_Flow_rate_at_level_5;//HW_Setting1.Flour_Flow_rate_at_level_5;//Flour_Flow_rate_at_level_5;
				if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_5)
				{
					Flour_Stepper_Motor.total_no_of_steps =  Flour_Stepper_Motor.total_no_of_steps;//4809+3500+3470//93.33 steps per gram
				}
				else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_4 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_5)
				{
					Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps* flourFeedrate.feedrate1;//1.04;//4992//183
				}
				else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_3 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_4)
				{
					Flour_Stepper_Motor.total_no_of_steps =  Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate2;//1.06;//5120//311
				}
				else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_2 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_3)
				{
					Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate3;//1.10;//5306//497
				}
				else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_1 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_2)
				{
					Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate4;//1.14;//5524//715
				}
				else
				{
					Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate5;//1.14;
				}
				Flour_stepper_motor_state = Flour_stepper_motor_init;
				kneadingProcessState = kneading_dispense_water_qty1;
				Flour_Stepper_Motor.steps_counter=0;
			}
			break;
		case kneading_dispense_water_qty1:
			if(Flour_stepper_motor_state == Flour_stepper_motor_completed)
			{
				Flour_stepper_motor_state = Flour_stepper_motor_idle;
				//Water quantity to be dispensed
				//Update the number of steps for motor to rotate to dispense
				Water_Stepper_Motor.total_no_of_steps = (HW_Setting1.Water_Pump_Flow_Rate + WATER_DISPENSE_STEPS_TOLERANCE) * Ingredient_mixing_parameters.Water_Qty_1;
				//Update the number of pulses for motor to rotate to dispense for flow sensor feedback
				water_value = Ingredient_mixing_parameters.Water_Qty_1 * WATER_PULSE_VALUE;
				Water_stepper_motor_state = Water_stepper_motor_init;
				kneadingProcessState = kneading_dispense_oil_qty1;
			}
			break;
		case kneading_dispense_oil_qty1:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				Water_stepper_motor_state = Water_stepper_motor_idle;
				//Update the number of steps for motor to rotate to dispense
				Oil_Stepper_Motor.total_no_of_steps = HW_Setting1.Oil_Pump_Flow_Rate * Ingredient_mixing_parameters.Oil_Qty1;
				//Oil quantity to be dispensed
				//oilDispensingParams->quantityToDispense = Ingredient_mixing_parameters.Oil_Qty1;
				Oil_stepper_motor_state = Oil_stepper_motor_init;
				kneadingProcessState = kneading_dispense_wait_to_oil_qty1_complete;
			}
			break;
		case kneading_dispense_wait_to_oil_qty1_complete:
			if(Oil_stepper_motor_state == Oil_stepper_motor_completed)
			{
				Oil_stepper_motor_state = Oil_stepper_motor_idle;
				//Update the mixing time
				mixingTimeValuefromStructure = Ingredient_mixing_parameters.Mixing_time_1;
				timerCount.kneadingTimerCnt = 0;
				kneadingProcessState = kneading_mixing_time1;
			}
			break;
		case kneading_mixing_time1:		//Down 1 after 1st dispense
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				//Update the distance for movement of kneader leadscrew motor
				kneadLeadscrewTravel.newMMTravel = (Ingredient_mixing_parameters.Dough_Base_Step_1 /** leadScrewMotor.stepPerMMRotation*/);			//Distance b/w blade and base of dough
				leadscrewMotorKneadingDirection = CLOCKWISE;
				Knead_movement_to_xx_mm_state = knead_screw_start;
				kneadingProcessState = kneading_leadscrew_step1;
			}
			break;
		case kneading_leadscrew_step1:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				if(Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_1 > 0)
				{
					kneadingProcessState = kneading_rounding_clockwise1_time;
					Toggle_motor_main_state = Toggle_motor_idle;
				}
				else
				{
					kneadingProcessState = kneading_dispense_flour_qty2;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
			}
			break;
		case kneading_rounding_clockwise1_time:
			//Toggle motor clockwise for one count with updated clockwise on time
			Toggle_motor_main_state = Toggle_clockwise;
			Toggle_motor_clockwise = Toggle_loop_check;
			toggle_motor_clockwise_counts = 1;
			Clockwise_on_time = Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_1;
			Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
			timerCount.togglingTimerCnt = 0;
			kneadingProcessState = kneading_dispense_flour_qty2;
			break;
		case kneading_dispense_flour_qty2:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 10;
				Toggling_DC_Motor_Stop();
				//Update the number of steps for motor to rotate to dispense

#if 		FLOUR_PRESENCE_SENSOR
				if(pnpProximityValues.flourConnectorPresence == 1)
#endif
				{
					Flour_Stepper_Motor.total_no_of_steps =Ingredient_mixing_parameters.Flour_Qty2* HW_Setting1.Flour_Flow_rate_at_level_5;//HW_Setting1.Flour_Flow_rate_at_level_5;//Flour_Flow_rate_at_level_5;
					if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_5)
					{
						Flour_Stepper_Motor.total_no_of_steps =  Flour_Stepper_Motor.total_no_of_steps;//4809+3500+3470//93.33 steps per gram
					}
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_4 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_5)
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate1;//1.05;//3675//183
					}
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_3 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_4)
					{
						Flour_Stepper_Motor.total_no_of_steps =  Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate2;//1.08;//5120//311
					}
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_2 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_3)
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate3;//1.13;//5306//497
					}
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_1 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_2)
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate4; //1.20;//5524//715
					}
					else
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate5;//1.20;
					}
					Toggle_motor_main_state = Toggle_motor_idle;
					Toggle_motor_clockwise = Toggle_motor_end;
					Flour_stepper_motor_state = Flour_stepper_motor_init;
					kneadingProcessState = kneading_dispense_water_qty2;
				}
			}
			break;
		case kneading_dispense_water_qty2:
			if(Flour_stepper_motor_state == Flour_stepper_motor_completed)
			{
				Flour_stepper_motor_state = Flour_stepper_motor_idle;
				//Water quantity to be dispensed
				//Update the number of steps for motor to rotate to dispense
				Water_Stepper_Motor.total_no_of_steps = (HW_Setting1.Water_Pump_Flow_Rate + WATER_DISPENSE_STEPS_TOLERANCE) * Ingredient_mixing_parameters.Water_Qty_2;
				water_value = Ingredient_mixing_parameters.Water_Qty_2 * WATER_PULSE_VALUE;
				Water_stepper_motor_state = Water_stepper_motor_init;
				kneadingProcessState = kneading_dispense_oil_qty2;
			}
			break;
		case kneading_dispense_oil_qty2:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				Water_stepper_motor_state = Water_stepper_motor_idle;
				//Oil quantity to be dispensed
				//Update the number of steps for motor to rotate to dispense
				Oil_Stepper_Motor.total_no_of_steps = HW_Setting1.Oil_Pump_Flow_Rate * Ingredient_mixing_parameters.Oil_Qty2;
				Oil_stepper_motor_state = Oil_stepper_motor_init;
				kneadingProcessState = kneading_dispense_wait_to_oil_qty2_complete;
				timerCount.kneadingTimerCnt = 0;
			}
			break;
			//Check for completion of steps first & then start the mixing time
		case kneading_dispense_wait_to_oil_qty2_complete:
			if(Oil_stepper_motor_state == Oil_stepper_motor_completed)
			{
				Oil_stepper_motor_state = Oil_stepper_motor_idle;
				//Update the mixing time
				mixingTimeValuefromStructure = Ingredient_mixing_parameters.Mixing_time_2;
				timerCount.kneadingTimerCnt = 0;
				kneadingProcessState = kneading_mixing_time2;
			}
			break;
		case kneading_mixing_time2:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				kneadLeadscrewTravel.newMMTravel = (Ingredient_mixing_parameters.Dough_Base_Step_2 /** leadScrewMotor.stepPerMMRotation*/);			//Distance b/w blade and base of dough
				leadscrewMotorKneadingDirection = CLOCKWISE;
				Knead_movement_to_xx_mm_state = knead_screw_start;
				kneadingProcessState = kneading_leadscrew_step2;
			}
			break;
		case kneading_leadscrew_step2:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				if(Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_2 > 0)
				{
					kneadingProcessState = kneading_rounding_clockwise2_time;
					Toggle_motor_main_state = Toggle_motor_idle;
				}
				else
				{
					kneadingProcessState = kneading_dispense_flour_qty3;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
			}
			break;
		case kneading_rounding_clockwise2_time:
			//Rotate toggle motor clockwise direction for required time
			Toggle_motor_main_state = Toggle_clockwise;
			Toggle_motor_clockwise = Toggle_loop_check;
			toggle_motor_clockwise_counts = 1;
			Clockwise_on_time = Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_2;
			Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
			timerCount.togglingTimerCnt = 0;
			kneadingProcessState = kneading_dispense_flour_qty3;
			break;
		case kneading_dispense_flour_qty3:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 15;
				Toggling_DC_Motor_Stop();
				//Update the number of steps for motor to rotate to dispense
#if 		FLOUR_PRESENCE_SENSOR
				if(pnpProximityValues.flourConnectorPresence == 1)
#endif
				{
					Flour_Stepper_Motor.total_no_of_steps =Ingredient_mixing_parameters.Flour_Qty3 * HW_Setting1.Flour_Flow_rate_at_level_5;//HW_Setting1.Flour_Flow_rate_at_level_5;//Flour_Flow_rate_at_level_5;
					if(flourLoadCellValue->loadCellValueAfterTare > FLOUR_DISPENSE_LEVEL1)
					//if(flourLoadCellValue->loadCellValueAfterTare > 6000)
					{
						Flour_Stepper_Motor.total_no_of_steps =  Flour_Stepper_Motor.total_no_of_steps;//4809+3500+3470//93.33 steps per gram
					}
					//else if(flourLoadCellValue->loadCellValueAfterTare > 4000 && flourLoadCellValue->loadCellValueAfterTare < 6000)
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_4 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_5)
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate1;//1.05;//3675//183
					}
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_3 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_4)
					{
						Flour_Stepper_Motor.total_no_of_steps =  Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate2; //1.08;//5120//311
					}
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_2 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_3)
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate3;//1.13;//5306//497
					}
					else if(flourLoadCellValue->loadCellValueAfterTare > HW_Setting1.Flour_Level_1 && flourLoadCellValue->loadCellValueAfterTare < HW_Setting1.Flour_Level_2)
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate4; //1.20;//5524//715
					}
					else
					{
						Flour_Stepper_Motor.total_no_of_steps = Flour_Stepper_Motor.total_no_of_steps * flourFeedrate.feedrate5; //1.20;
					}
					Flour_stepper_motor_state = Flour_stepper_motor_init;
					Toggle_motor_main_state = Toggle_motor_idle;
					Toggle_motor_clockwise = Toggle_motor_end;
					kneadingProcessState = kneading_dispense_water_qty3;
				}
			}
			else
			{
				kneadingProcessState = kneading_dispense_flour_qty3;
			}
			break;
		case kneading_dispense_water_qty3:
			if(Flour_stepper_motor_state == Flour_stepper_motor_completed)
			{
				Flour_stepper_motor_state = Flour_stepper_motor_idle;
				//Water quantity to be dispensed
				//Update the number of steps for motor to rotate to dispense
				Water_Stepper_Motor.total_no_of_steps = (HW_Setting1.Water_Pump_Flow_Rate + WATER_DISPENSE_STEPS_TOLERANCE) * Ingredient_mixing_parameters.Water_Qty_3;
				water_value = Ingredient_mixing_parameters.Water_Qty_3 * WATER_PULSE_VALUE;
				Water_stepper_motor_state = Water_stepper_motor_init;
				kneadingProcessState = kneading_dispense_oil_qty3;
			}
			break;
		case kneading_dispense_oil_qty3:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				Water_stepper_motor_state = Water_stepper_motor_idle;
				//Oil quantity to be dispensed
				//Update the number of steps for motor to rotate to dispense
				Oil_Stepper_Motor.total_no_of_steps = HW_Setting1.Oil_Pump_Flow_Rate * Ingredient_mixing_parameters.Oil_Qty3;
				Oil_stepper_motor_state = Oil_stepper_motor_init;
				mixingTimeValuefromStructure = Ingredient_mixing_parameters.Mixing_time_3;
				timerCount.kneadingTimerCnt = 0;
				kneadingProcessState=kneading_dispense_wait_to_oil_qty3_complete;
			}
			break;
			//Check for Oil dispensing to be completed, then start the mixing time
		case kneading_dispense_wait_to_oil_qty3_complete:
			if(Oil_stepper_motor_state == Oil_stepper_motor_completed)
			{
				Oil_stepper_motor_state = Oil_stepper_motor_idle;
				mixingTimeValuefromStructure = Ingredient_mixing_parameters.Mixing_time_3;
				timerCount.kneadingTimerCnt = 0;
				kneadingProcessState = kneading_mixing_time3;
			}
			break;
		case kneading_mixing_time3:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				kneadLeadscrewTravel.newMMTravel = (Ingredient_mixing_parameters.Dough_Base_Step_3 /** leadScrewMotor.stepPerMMRotation*/);			//Distance b/w blade and base of dough
				leadscrewMotorKneadingDirection = CLOCKWISE;
				Knead_movement_to_xx_mm_state = knead_screw_start;
				kneadingProcessState = kneading_leadscrew_step3;

			}
			break;
		case kneading_leadscrew_step3:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				if(Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_3 > 0)
				{
					kneadingProcessState = kneading_rounding_clockwise3_time;
					Toggle_motor_main_state = Toggle_motor_idle;
				}
				else
				{
					kneadingProcessState = kneading_dispense_water_qty4;
					Toggle_motor_main_state = Toggle_motor_completed;
				}
			}
			break;
		case kneading_rounding_clockwise3_time:
			Toggle_motor_main_state = Toggle_clockwise;
			Toggle_motor_clockwise = Toggle_loop_check;
			toggle_motor_clockwise_counts = 1;
			Clockwise_on_time = Ingredient_mixing_parameters.Rounding_Motor_Clock_Wise_time_3;
			Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
			timerCount.togglingTimerCnt = 0;
			kneadingProcessState = kneading_dispense_water_qty4;
			break;
		case kneading_dispense_water_qty4:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				Toggling_DC_Motor_Stop();
				Toggle_motor_main_state = Toggle_motor_idle;
				Toggle_motor_clockwise = Toggle_motor_end;
				Oil_stepper_motor_state = Oil_stepper_motor_idle;
				Flour_stepper_motor_state = Oil_stepper_motor_idle;
				//Flour dispense quantity to be added
				//Update the number of steps for motor to rotate to dispense
				Water_Stepper_Motor.total_no_of_steps  = (HW_Setting1.Water_Pump_Flow_Rate + 5) * Ingredient_mixing_parameters.Water_Qty_4;
				water_value = Ingredient_mixing_parameters.Water_Qty_4 * WATER_PULSE_VALUE;
				Water_stepper_motor_state = Water_stepper_motor_init;
				kneadingProcessState = kneading_mixing_time4;
				timerCount.kneadingTimerCnt = 0;
			}
			break;
		case kneading_mixing_time4:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				Toggling_DC_Motor_Stop();
				mixingTimeValuefromStructure = Ingredient_mixing_parameters.Mixing_time_4;
				timerCount.kneadingTimerCnt = 0;
				Water_stepper_motor_state = Water_stepper_motor_idle;
				kneadingProcessState = kneading_dispense_water_qty5;
			}
			break;
		case kneading_dispense_water_qty5:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				dispenseStateForInterrupt = waterDispense;
				//Update the number of steps for motor to rotate to dispense
				Water_Stepper_Motor.total_no_of_steps = (HW_Setting1.Water_Pump_Flow_Rate + WATER_DISPENSE_STEPS_TOLERANCE) * Ingredient_mixing_parameters.Water_Qty_5;
				water_value = Ingredient_mixing_parameters.Water_Qty_5 * WATER_PULSE_VALUE;
				Water_stepper_motor_state = Water_stepper_motor_init;
				timerCount.kneadingTimerCnt = 0;
				kneadingProcessState = kneading_mixing_time5;
			}
			break;
		case kneading_mixing_time5:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				kneading_process_percent = 20;
				Water_stepper_motor_state = Water_stepper_motor_idle;
				mixingTimeValuefromStructure = Ingredient_mixing_parameters.Mixing_time_5;
				timerCount.kneadingTimerCnt = 0;
				kneadingProcessState = kneading_leadscrew_step4;
			}
			break;
		case kneading_leadscrew_step4:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneadLeadscrewTravel.newMMTravel = (Kneading_parameters.Dough_Base_Step_4 /** leadScrewMotor.stepPerMMRotation*/);			//Distance b/w blade and base of dough
				leadscrewMotorKneadingDirection = CLOCKWISE;
				Knead_movement_to_xx_mm_state = knead_screw_start;
				kneadingProcessState = kneading_rounding_clockwise1;
			}
			break;
			/*Ingredient Mixing Cycle Complete*/
		case kneading_rounding_clockwise1:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				if(Kneading_parameters.Rounding_Motor_Clock_Wise_time_4 > 0)
				{
					Toggle_motor_main_state = Toggle_clockwise;
					Toggle_motor_clockwise = Toggle_loop_check;
					toggle_motor_clockwise_counts = 1;
					Clockwise_on_time = Kneading_parameters.Rounding_Motor_Clock_Wise_time_4;
					Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
					timerCount.togglingTimerCnt = 0;
				}
				else
				{
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				kneadingProcessState = kneading_rounding_clockwise1_wait_to_complete;
			}
			break;
		case kneading_rounding_clockwise1_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 25;
				Toggling_DC_Motor_Stop();
				timerCount.togglingTimerCnt = 0;
				Toggle_motor_main_state = Toggle_motor_idle;
				kneadingProcessState = kneading_start_blade_motor_at_second_rpm;
			}
			break;
		case kneading_start_blade_motor_at_second_rpm:
			kneading_state_android = 4;
			Toggling_DC_Motor_Stop();
			/*Added for testing of proper kneading output*/
#if 			bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Kneading_parameters.Kneading_Speed_1;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;
				Kneading_motor_state = Kneading_motor_init;
			}
#endif
			timerCount.kneadingTimerCnt = 0;
			mixingTimeValuefromStructure = Kneading_parameters.Kneading_Time_1;
			kneadingProcessState = kneading_blade_motor_first_mixing_time;
			break;
		case kneading_blade_motor_first_mixing_time:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				kneadingProcessState = kneading_start_blade_motor_at_third_rpm;
			}
			break;
		case kneading_start_blade_motor_at_third_rpm:

#if 			bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Kneading_parameters.Kneading_Speed_2;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;		//Commented
				Kneading_motor_state = Kneading_motor_init;
			}
#endif
			Toggling_DC_Motor_Stop();
			leadscrewMotorKneadingDirection = ANTICLOCKWISE;
			kneadLeadscrewTravel.newMMTravel = (Kneading_parameters.Dough_Base_Step_5 /** leadScrewMotor.stepPerMMRotation*/);			//Distance b/w blade and base of dough
			Knead_movement_to_xx_mm_state = knead_screw_start;
			kneadingProcessState = kneading_leadscrew_step5;
			break;
		case kneading_leadscrew_step5:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Toggling_DC_Motor_Stop();
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneadingProcessState = kneading_rounding_clockwise2;
			}
			break;
		case kneading_rounding_clockwise2:
			if(Kneading_parameters.Rounding_Motor_Clock_Wise_time_5 > 0)
			{
				Toggle_motor_main_state = Toggle_clockwise;
				Toggle_motor_clockwise = Toggle_loop_check;
				toggle_motor_clockwise_counts = 1;
				Clockwise_on_time = Kneading_parameters.Rounding_Motor_Clock_Wise_time_5;
				Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
				timerCount.togglingTimerCnt = 0;
			}
			else
			{
				Toggle_motor_main_state = Toggle_motor_completed;
			}
			kneadingProcessState = kneading_rounding_clockwise2_wait_to_complete;
			break;
		case kneading_rounding_clockwise2_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 30;
				Toggling_DC_Motor_Stop();
				timerCount.togglingTimerCnt = 0;
				Toggle_motor_main_state = Toggle_motor_idle;
				timerCount.kneadingTimerCnt = 0;
				mixingTimeValuefromStructure = Kneading_parameters.Kneading_Time_2;
				kneadingProcessState = kneading_blade_motor_second_mixing_time;
			}
			break;
		case kneading_blade_motor_second_mixing_time:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				kneadLeadscrewTravel.newMMTravel = (Kneading_parameters.Dough_Base_Step_6 /** leadScrewMotor.stepPerMMRotation*/);			//Distance b/w blade and base of dough
				leadscrewMotorKneadingDirection = CLOCKWISE;
				Knead_movement_to_xx_mm_state = knead_screw_start;
				kneadingProcessState = kneading_leadscrew_step6;
			}
			else
			{
				kneadingProcessState = kneading_blade_motor_second_mixing_time;
			}
			break;
		case kneading_leadscrew_step6:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Toggling_DC_Motor_Stop();
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneadingProcessState = kneading_rounding_clockwise3;
			}
			else
			{
				kneadingProcessState = kneading_leadscrew_step6;
			}
			break;
		case kneading_rounding_clockwise3:
			if(Kneading_parameters.Rounding_Motor_Clock_Wise_time_6 > 0)
			{
				Toggle_motor_main_state = Toggle_clockwise;
				Toggle_motor_clockwise = Toggle_loop_check;
				toggle_motor_clockwise_counts = 1;
				Clockwise_on_time = Kneading_parameters.Rounding_Motor_Clock_Wise_time_6;
				Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
				timerCount.togglingTimerCnt = 0;
			}
			else
			{
				Toggle_motor_main_state = Toggle_motor_completed;
			}
			kneadingProcessState = kneading_rounding_clockwise3_wait_to_complete;
			break;
		case kneading_rounding_clockwise3_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 35;
				Toggling_DC_Motor_Stop();
				timerCount.togglingTimerCnt = 0;
				Toggle_motor_main_state = Toggle_motor_idle;
				kneadingProcessState = kneading_start_blade_motor_at_fourth_rpm;
			}
			break;
		case kneading_start_blade_motor_at_fourth_rpm:
			/*Added for testing of proper kneading output*/
#if 			bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Kneading_parameters.Kneading_Speed_3;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;
				Kneading_motor_state = Kneading_motor_init;
			}
#endif
			/*Added for testing of proper kneading output*/
			Toggling_DC_Motor_Stop();
			timerCount.kneadingTimerCnt = 0;
			mixingTimeValuefromStructure = Kneading_parameters.Kneading_Time_3;
			kneadingProcessState = kneading_blade_motor_third_mixing_time;
			break;
		case kneading_blade_motor_third_mixing_time:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				kneadingProcessState = kneading_start_blade_motor_at_fifth_rpm;
			}
			else
			{
				kneadingProcessState = kneading_blade_motor_third_mixing_time;
			}
			break;
		case kneading_start_blade_motor_at_fifth_rpm:
			/*Added for testing of proper kneading output*/
#if 			bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Kneading_parameters.Kneading_Speed_4;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;
				Kneading_motor_state = Kneading_motor_init;
			}
#endif
			/*Added for testing of proper kneading output*/
			Toggling_DC_Motor_Stop();
			kneadLeadscrewTravel.newMMTravel = Kneading_parameters.Dough_Base_Step_7/* * leadScrewMotor.stepPerMMRotation*/;
			leadscrewMotorKneadingDirection = ANTICLOCKWISE;
			Knead_movement_to_xx_mm_state = knead_screw_start;
			kneadingProcessState = kneading_leadscrew_step7;
			break;
		case kneading_leadscrew_step7:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Toggling_DC_Motor_Stop();
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneadingProcessState = kneading_rounding_clockwise4;
			}
			else
			{
				kneadingProcessState = kneading_leadscrew_step7;
			}
			break;

		case kneading_rounding_clockwise4:
			if(Kneading_parameters.Rounding_Motor_Clock_Wise_time_7 > 0)
			{
				Toggle_motor_main_state = Toggle_clockwise;
				Toggle_motor_clockwise = Toggle_loop_check;
				toggle_motor_clockwise_counts = 1;
				Clockwise_on_time = Kneading_parameters.Rounding_Motor_Clock_Wise_time_7;
				Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
				timerCount.togglingTimerCnt = 0;
			}
			else
			{
				Toggle_motor_main_state = Toggle_motor_completed;
			}
			kneadingProcessState = kneading_rounding_clockwise4_wait_to_complete;
			break;
		case kneading_rounding_clockwise4_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 40;
				Toggling_DC_Motor_Stop();
				timerCount.togglingTimerCnt = 0;
				toggle_count=0;
				Toggle_motor_main_state = Toggle_motor_idle;
				timerCount.kneadingTimerCnt = 0;
				mixingTimeValuefromStructure = Kneading_parameters.Kneading_Time_4;
				kneadingProcessState = kneading_blade_motor_fourth_mixing_time;
			}
			break;
		case kneading_blade_motor_fourth_mixing_time:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				kneadLeadscrewTravel.newMMTravel = Kneading_parameters.Dough_Base_Step_8 /** leadScrewMotor.stepPerMMRotation*/;
				leadscrewMotorKneadingDirection = CLOCKWISE;
				Knead_movement_to_xx_mm_state = knead_screw_start;
				kneadingProcessState = kneading_leadscrew_step8;
			}
			else
			{
				kneadingProcessState = kneading_blade_motor_fourth_mixing_time;
			}
			break;
		case kneading_leadscrew_step8:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Toggling_DC_Motor_Stop();
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneadingProcessState = kneading_rounding_clockwise5;
			}
			else
			{
				kneadingProcessState = kneading_leadscrew_step8;
			}
			break;
		case kneading_rounding_clockwise5:
			if(Kneading_parameters.Rounding_Motor_Clock_Wise_time_8 > 0)
			{
				Toggle_motor_main_state = Toggle_clockwise;
				Toggle_motor_clockwise = Toggle_loop_check;
				toggle_motor_clockwise_counts = 1;
				Clockwise_on_time = Kneading_parameters.Rounding_Motor_Clock_Wise_time_8;
				Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
				timerCount.togglingTimerCnt = 0;
			}
			else
			{
				Toggle_motor_main_state = Toggle_motor_completed;
			}
			kneadingProcessState = kneading_rounding_clockwise5_wait_to_complete;
			break;
		case kneading_rounding_clockwise5_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 45;
				Toggling_DC_Motor_Stop();
				timerCount.togglingTimerCnt = 0;
				Toggle_motor_main_state = Toggle_motor_idle;
				kneadingProcessState = kneading_start_blade_motor_at_sixth_rpm;
			}
			break;
		case kneading_start_blade_motor_at_sixth_rpm:
			/*Added for testing of proper kneading output*/
#if 			bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Kneading_parameters.Kneading_Speed_5;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;		//90
				Kneading_motor_state = Kneading_motor_init;
			}
#endif
			/*Added for testing of proper kneading output*/
			Toggling_DC_Motor_Stop();
			timerCount.kneadingTimerCnt = 0;
			mixingTimeValuefromStructure = Kneading_parameters.Kneading_Time_5;
			kneadingProcessState = kneading_blade_motor_fifth_mixing_time;
			break;
		case kneading_blade_motor_fifth_mixing_time:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				kneadingProcessState = kneading_start_blade_motor_at_seventh_rpm;
			}
			else
			{
				kneadingProcessState = kneading_blade_motor_fifth_mixing_time;
			}
			break;
		case kneading_start_blade_motor_at_seventh_rpm:
			/*Added for testing of proper kneading output*/
#if 			bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Kneading_parameters.Kneading_Speed_6;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;		//Commented
				Kneading_motor_state = Kneading_motor_init;
			}
#endif
			/*Added for testing of proper kneading output*/
			Toggling_DC_Motor_Stop();
			leadscrewMotorKneadingDirection = ANTICLOCKWISE;
			kneadLeadscrewTravel.newMMTravel = (Kneading_parameters.Dough_Base_Step_9 /** leadScrewMotor.stepPerMMRotation*/);
			Knead_movement_to_xx_mm_state = knead_screw_start;
			kneadingProcessState = kneading_leadscrew_step9;
			break;
		case kneading_leadscrew_step9:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				Toggling_DC_Motor_Stop();
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneadingProcessState = kneading_rounding_clockwise6;
			}
			else
			{
				kneadingProcessState = kneading_leadscrew_step9;
			}
			break;
		case kneading_rounding_clockwise6:
			if(Kneading_parameters.Rounding_Motor_Clock_Wise_time_9 > 0)
			{
				Toggle_motor_main_state = Toggle_clockwise;
				Toggle_motor_clockwise = Toggle_loop_check;
				toggle_motor_clockwise_counts = 1;
				Clockwise_on_time = Kneading_parameters.Rounding_Motor_Clock_Wise_time_9;
				Toggle_off_time_android = TOGGLE_DEFAULT_OFFTIME_MS;
				timerCount.togglingTimerCnt = 0;
			}
			else
			{
				Toggle_motor_main_state = Toggle_motor_completed;
			}
			kneadingProcessState = kneading_rounding_clockwise6_wait_to_complete;
			break;
		case kneading_rounding_clockwise6_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 50;
				Toggling_DC_Motor_Stop();
				timerCount.togglingTimerCnt = 0;
				Toggle_motor_main_state = Toggle_motor_idle;
				timerCount.kneadingTimerCnt = 0;
				mixingTimeValuefromStructure = Kneading_parameters.Kneading_Time_6;
				kneadingProcessState = kneading_blade_motor_sixth_mixing_time;
			}
			break;
		case kneading_blade_motor_sixth_mixing_time:
			if(timerCount.kneadingTimerCnt >= mixingTimeValuefromStructure)
			{
				Toggling_DC_Motor_Stop();
				//					 Knead_screw_Stepper_Motor.total_no_of_steps = 0;
				leadscrewMotorKneadingDirection = CLOCKWISE;
				kneadLeadscrewTravel.newMMTravel = (Coating_Rounding.Dough_Base_Step_10/* * leadScrewMotor.stepPerMMRotation*/);
				Knead_movement_to_xx_mm_state = knead_screw_start;
				kneadingProcessState = kneading_leadscrew_step10;
			}
			break;
		case kneading_leadscrew_step10:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				kneading_process_percent = 55;
				Toggling_DC_Motor_Stop();
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				kneading_state_android = 5;
				kneadingProcessState = kneading_round_coating_speed_rpm;
			}
			else
			{
				//					kneading_process_percent = 80;
				kneading_state_android = 5;
				kneadingProcessState = kneading_leadscrew_step10;
			}
			break;
			/*Kneading Process Complete*/

		case kneading_round_coating_speed_rpm:
			Toggling_DC_Motor_Stop();
			Toggle_motor_main_state = Toggle_motor_idle;

#if 			bladePWM
			bladeMotorSpeedValueFromStructure = SPEED_CONVERTER_VALUE * Coating_Rounding.Coating_Speed;
			if(bladeMotorSpeedValueFromStructure > 0 )
			{
				currentBladePWM = bladeMotorSpeedValueFromStructure;
				Kneading_motor_state = Kneading_motor_init;
			}
#endif
			kneadingProcessState = kneading_pre_coating_rounding_clockwise;
			break;

		case kneading_pre_coating_rounding_clockwise:
			toggleMotorClockAntiClockCnt = Coating_Rounding.pre_coating_clockwise_anticlockwise_count;
			Clockwise_on_time = Coating_Rounding.pre_coating_clockwise_time;
			Anti_Clockwise_on_time = Coating_Rounding.pre_coating_anticlockwise_time;
			Toggle_off_time_android = Coating_Rounding.pre_coating_delay_between_shifts;
			timerCount.togglingTimerCnt = 0;
			Toggle_motor_main_state = Toggle_clockwise_anticlockwise;
			Toggle_motor_clockwise_anticlockwise = Toggle_loop_check;
			kneadingProcessState = kneading_pre_coating_rounding_clockwise_wait_to_complete;
			break;
		case kneading_pre_coating_rounding_clockwise_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggleMotorClockAntiClockCnt = 0;
				Clockwise_on_time = 0;
				Anti_Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 60;
				Toggling_DC_Motor_Stop();
#if 		FLOUR_PRESENCE_SENSOR
				if(pnpProximityValues.flourConnectorPresence == 1)
#endif
				{
					Toggle_motor_clockwise_anticlockwise = Toggle_motor_end;
					Toggle_motor_main_state = Toggle_motor_idle;
					Flour_Stepper_Motor.total_no_of_steps = Coating_Rounding.Flour_Coating_qty * HW_Setting1.Flour_Flow_rate_at_level_5;//;600;//350
					Flour_stepper_motor_state = Flour_stepper_motor_init;
					kneadingProcessState = kneading_round_flour_coating;
				}
			}
			break;
		case kneading_round_flour_coating:
			if(Flour_stepper_motor_state == Flour_stepper_motor_completed)
			{
#if				bladePWM
				currentBladePWM = 0;
				Kneading_motor_state = Kneading_motor_init;
#elif			!bladePWM
				HAL_GPIO_WritePin(DC_SSR_OP1_GPIO_Port, DC_SSR_OP1_Pin, RESET);
#endif
				Flour_stepper_motor_state = Flour_stepper_motor_idle;
				Oil_stepper_motor_state = Oil_stepper_motor_idle;
				Water_Stepper_Motor.total_no_of_steps=Coating_Rounding.water_coating_qty*HW_Setting1.Water_Pump_Flow_Rate;
				Water_stepper_motor_state = Water_stepper_motor_init;
				kneadingProcessState = kneading_round_water_coating;
			}
			else
			{
				kneadingProcessState = kneading_round_flour_coating;
			}
			break;
		case kneading_round_water_coating:
			if(Water_stepper_motor_state == Water_stepper_motor_completed)
			{
				Water_stepper_motor_state = Water_stepper_motor_idle;
				Oil_Stepper_Motor.total_no_of_steps= Coating_Rounding.oil_coating_qty*HW_Setting1.Oil_Pump_Flow_Rate;
				Oil_stepper_motor_state = Oil_stepper_motor_init;
				kneadingProcessState = kneading_round_oil_coating;
			}
			else
			{
				kneadingProcessState = kneading_round_water_coating;
			}
			break;
		case kneading_round_oil_coating:
			if(Oil_stepper_motor_state == Oil_stepper_motor_completed)
			{
				kneadingProcessState = kneading_blade_motor_off_wait;
			}
			break;
		case kneading_blade_motor_off_wait:
#if			!KNEADING_TIME_REDUCE
			if(Kneading_motor_state == Kneading_motor_completed)
#endif
			{
				if(Coating_Rounding.Rouding_Clock_wise_cycles >= 0)
				{
					toggle_motor_clockwise_counts = Coating_Rounding.Rouding_Clock_wise_cycles;
					Clockwise_on_time = Coating_Rounding.Rounding_time;
					Toggle_off_time_android = Coating_Rounding.delay_between_cycles;
					timerCount.togglingTimerCnt = 0;
					toggle_count = 0;//
					togglePWMDutyCycle=0;//
					Toggle_motor_main_state = Toggle_clockwise;
					Toggle_motor_clockwise = Toggle_loop_check;
				}
				else
				{
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				kneadingProcessState = kneading_rounding_clockwise_count;
			}
			break;
		case kneading_rounding_clockwise_count:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggle_motor_clockwise_counts = 0;
				Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 70;

				Toggling_DC_Motor_Stop();
				if(Coating_Rounding.Roudnig_motor_clockwise_and_anitclockwise_cycles >= 0)
				{
					toggleMotorClockAntiClockCnt = Coating_Rounding.Roudnig_motor_clockwise_and_anitclockwise_cycles;
					Clockwise_on_time = Coating_Rounding.clockwise_time;
					Anti_Clockwise_on_time = Coating_Rounding.Anti_clock_wise_time;
					Toggle_off_time_android = Coating_Rounding.delay_between_directions_shift;
					timerCount.togglingTimerCnt = 0;
					Toggle_motor_main_state = Toggle_clockwise_anticlockwise;
					Toggle_motor_clockwise_anticlockwise = Toggle_loop_check;
				}
				else
				{
					Toggle_motor_main_state = Toggle_motor_completed;
				}
				kneadingProcessState = kneading_rounding_clockwise_anticlockwise_count_wait_to_complete;
			}
			break;
		case kneading_rounding_clockwise_anticlockwise_count_wait_to_complete:
			if(Toggle_motor_main_state == Toggle_motor_completed)
			{
				toggleMotorClockAntiClockCnt = 0;
				Clockwise_on_time = 0;
				Anti_Clockwise_on_time = 0;
				Toggle_off_time_android = 0;
				kneading_process_percent = 80;
				kneading_state_android = 6;
				Toggling_DC_Motor_Stop();
#if 	!doughReleaseEnable
				/*If pizza Quantity given is one and completed pizza kneading is 0, move the knead base to bottom*/
				if((Pizza_setting.quantity == 1)&&(pizza_quantity==0))
				{
					Toggle_motor_main_state = Toggle_motor_idle;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
					kneadingProcessState = kneading_complete;
				}
				/*If pizza Quantity given is greater than one and completed pizza kneading is less than actual quantity, move the knead base to bottom*/
				else if((pizza_quantity==0)&&(Pizza_setting.quantity > 1 &&  pizza_quantity < Pizza_setting.quantity ))
				{
					Toggle_motor_main_state = Toggle_motor_idle;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
					kneadingProcessState = kneading_complete;
				}
				/*If pizza Quantity given is greater than one and completed pizza kneading is less than actual quantity and the base is ejected out of the machine, move the knead base to bottom*/
				else if(Pizza_setting.quantity > 1 && dough_ejection_complete==1 && pizza_quantity < Pizza_setting.quantity )
				{
					Toggle_motor_main_state = Toggle_motor_idle;
					knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
					kneadingProcessState = kneading_complete;
				}
#elif	doughReleaseEnable
				kneadingProcessState = kneading_dough_release_start;
#endif
			}
			break;
		case kneading_dough_release_start:
			doughReleaseProcess = doughReleaseStart;
			kneadingProcessState = kneading_dough_release_wait_to_complete;
			break;
		case kneading_dough_release_wait_to_complete:
			if(doughReleaseProcess == doughReleaseComplete)
			{
				kneading_process_percent = 85;
				/*If pizza Quantity given is one and completed pizza kneading is 0, move the knead base to bottom*/
				if((Pizza_setting.quantity == 1)&&(pizza_quantity==0))
				{
					Toggle_motor_main_state = Toggle_motor_idle;
					leadscrewMotorKneadingDirection = CLOCKWISE;
					kneadLeadscrewTravel.newMMTravel = kneadLeadscrewTravel.maximumMMTravel - KNEADER_LEAD_SLOW_DISTANCE_ABV_PLATE;
					Knead_movement_to_xx_mm_state = knead_screw_start;
					kneadingProcessState = kneading_movement_x_mm_above_base_speed_reduce_wait_to_complete;
				}
				/*If pizza Quantity given is greater than one and completed pizza kneading is less than actual quantity, move the knead base to bottom*/
				else if((pizza_quantity==0)&&(Pizza_setting.quantity > 1 &&  pizza_quantity < Pizza_setting.quantity ))
				{
					Toggle_motor_main_state = Toggle_motor_idle;
					leadscrewMotorKneadingDirection = CLOCKWISE;
					kneadLeadscrewTravel.newMMTravel = kneadLeadscrewTravel.maximumMMTravel - KNEADER_LEAD_SLOW_DISTANCE_ABV_PLATE;
					Knead_movement_to_xx_mm_state = knead_screw_start;
					kneadingProcessState = kneading_movement_x_mm_above_base_speed_reduce_wait_to_complete;
				}
				/*If pizza Quantity given is greater than one and completed pizza kneading is less than actual quantity and the base is ejected out of the machine, move the knead base to bottom*/
				else if(Pizza_setting.quantity > 1 && dough_ejection_complete==1 && pizza_quantity < Pizza_setting.quantity )
				{
					Toggle_motor_main_state = Toggle_motor_idle;
					leadscrewMotorKneadingDirection = CLOCKWISE;
					kneadLeadscrewTravel.newMMTravel = kneadLeadscrewTravel.maximumMMTravel - KNEADER_LEAD_SLOW_DISTANCE_ABV_PLATE;
					Knead_movement_to_xx_mm_state = knead_screw_start;
					kneadingProcessState = kneading_movement_x_mm_above_base_speed_reduce_wait_to_complete;
				}
				doughReleaseProcess = doughReleaseIdle;
			}
			break;
		case kneading_movement_x_mm_above_base_speed_reduce_wait_to_complete:
			if(Knead_movement_to_xx_mm_state == knead_screw_completed)
			{
				kneading_process_percent = 95;
				Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
				Knead_movement_to_xx_mm_state = knead_screw_idle;
				knead_screw_rear_end_or_bottom_end_limit_state = knead_screw_start;
				kneadingProcessState = kneading_movement_slow_1mm_above_base_wait_to_complete;
			}
			break;
		case kneading_movement_slow_1mm_above_base_wait_to_complete:
			if(knead_screw_rear_end_or_bottom_end_limit_state == knead_screw_completed)
			{
				kneading_process_percent = 100;
				if(HW_Setting1.Kneading_leadscrew_motor_Speed > 0)
				{
					Knead_screw_Stepper_Motor.rpm = HW_Setting1.Kneading_leadscrew_motor_Speed;
				}
				else if(HW_Setting1.Kneading_leadscrew_motor_Speed == 0)
				{
					Knead_screw_Stepper_Motor.rpm = STEPPER_MOTOR_DEFAULT_RPM;
				}
				kneadingProcessState = kneading_complete;
			}
			break;
		case kneading_complete:
			kneading_process_percent = 100;
			kneading_state_android = 6;
			break;
		}
		osDelay(1);
  }
  /* USER CODE END Kneading_Flow_Task */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
	static int i=0;
	/* USER CODE BEGIN Callback01 */
	if(time_measure )
	{
		i++;
		if(i>=10)
		{
			i=0;
		}
	}
  /* USER CODE END Callback01 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance ==  TIM16)
  {
		gu16_TIM2_OVC++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
