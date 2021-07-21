/*
 * MAX_31855.c
 *
 *  Created on: Sep 23, 2020
 *      Author: Madhu
 */

#include"MAX_31855.h"

extern uint16_t Temperature_value_Sensor_top_pan,Temperature_value_Sensor_bottom_pan;

extern temperature_t kTypeTemperature;

Temp_status Top_pan_temperature_status,Bottom_pan_temperature_status;

//extern char temp_buff[200];

//extern uint32_t TempDelay;

extern uint8_t errorScreenFlag,tempScreenFlag;

extern uint16_t pTemp;

extern Pressing_Baking_t Pressing_Baking;

void Temperature_control(uint16_t Top_Pan_Cutoff,uint16_t Bottom_Pan_Cutoff)
{
	Top_pan_temperature_status = ReadTemperature(&kTypeTemperature.topPanTemperature,TOP_PAN);
	Bottom_pan_temperature_status = ReadTemperature(&kTypeTemperature.bottomPanTemperature,BOTTOM_PAN);
	if(kTypeTemperature.topPanTemperature == 128)
	{
		kTypeTemperature.topPanTemperature = 129;
	}
	if(kTypeTemperature.bottomPanTemperature == 128)
	{
		kTypeTemperature.bottomPanTemperature = 129;
	}
	if((kTypeTemperature.topPanTemperature < Top_Pan_Cutoff))
	{
		HAL_GPIO_WritePin(SSR_TOP_PAN_GPIO_Port, SSR_TOP_PAN_Pin, SET);
		//TOP PAN SSR_ON
	}
	else if((kTypeTemperature.topPanTemperature > Top_Pan_Cutoff))
	{
		HAL_GPIO_WritePin(SSR_TOP_PAN_GPIO_Port, SSR_TOP_PAN_Pin, RESET);
		//TOP PAN SSR_OFF
	}

	if((kTypeTemperature.bottomPanTemperature<Bottom_Pan_Cutoff))
	{
		HAL_GPIO_WritePin(SSR_BOTTOM_PAN_GPIO_Port, SSR_BOTTOM_PAN_Pin,SET);
		//BOTTOM PAN SSR_ON
	}
	else if((kTypeTemperature.bottomPanTemperature>Top_Pan_Cutoff))
	{
		HAL_GPIO_WritePin(SSR_BOTTOM_PAN_GPIO_Port, SSR_BOTTOM_PAN_Pin,RESET);
		//BOTTOM PAN SSR_OFF
	}

}

/************************************************************
 * Function: void TempError(Temp_status error)
 *
 * @brief:	 Function is called by PrintTempData() to print
 * 		   	 the thermocouple Fault condition
 *************************************************************/
void TempError(Temp_status error)
{
	if(error == TEMP_SCV_FAULT) /*thermocouple is short circuited to VCC*/
	{
		//		ILI9341_WriteString(ERROR_OFFSET,K_TYPE_Y_AXIS,"T:SC-VCC", Font_11x18, ILI9341_WHITE, ILI9341_WHITE);
		//		ILI9341_WriteString1(ERROR_OFFSET,K_TYPE_Y_AXIS,"T:SC-VCC", ILI9341_BLACK, ILI9341_WHITE);
	}
	else if(error == TEMP_SCG_FAULT) /*thermocouple is short circuited to GND*/
	{
		//		ILI9341_WriteString(ERROR_OFFSET, K_TYPE_Y_AXIS,"T:SC-GND", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
	}
	else if(error == TEMP_OC_FAULT) /*thermocouple is open*/
	{
		//		ILI9341_WriteString(ERROR_OFFSET,K_TYPE_Y_AXIS,"T:Open  ", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
	}
}


/**************************************************************
 * Function: Temp_status ReadTemperature(uint16_t *temp)
 *
 * @brief:	 Function is called by PrintTempData() to capture
 * 		   	 the thermocouple temperature and Fault condition
 *
 * @Note: 	 Update the HAL_GPIO_WritePin() function with
 * 			 the port value and pin value to which the SPI
 * 			 chip select is connected
 *
 * @Note: 	 SPI pheripheral to be configured with
 * 			 Mode      - Receive Only Master
 * 			 Data Size - 8Bits
 * 			 CPOL 	   - LOW
 * 			 CPHA 	   - 1 Edge
 * 			 and pass the appropriate SPI handle structure
 *
 * @Retval:  Temp_status value is returned
 *************************************************************/
Temp_status ReadTemperature(uint16_t *temp,Sensor_type sensor)
{
	uint8_t Temp_buf[5] = {"\0"};
	Temp_status status;
	if(sensor==TOP_PAN)
	{
		Temp_status status = TEMP_OK;

		/*Update this function with the port value and pin value
 		to which the SPI chip select is connected*/
		//HAL_GPIO_WritePin(MAX_SPI1_CS2_GPIO_Port, MAX_SPI1_CS2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MAX_SPI4_CS1_GPIO_Port, MAX_SPI4_CS1_Pin, GPIO_PIN_RESET);


	//HAL_Delay(5);
		/*Configure with the appropriate SPI handle structure*/
		HAL_SPI_Receive(&hspi4,Temp_buf,4,100);

		*temp = Temp_buf[0] << 8;

		*temp |= Temp_buf[1];

		HAL_GPIO_WritePin(MAX_SPI4_CS1_GPIO_Port, MAX_SPI4_CS1_Pin, GPIO_PIN_SET);

		if(*temp & 0x1)
		{
			if(Temp_buf[3] & 0x1) //Check to see if thermocouple is open
			{
				status = TEMP_OC_FAULT;
			}
			else if(Temp_buf[3] & 0x2) //Check to see if thermocouple is short circuited to GND
			{
				status = TEMP_SCG_FAULT;
			}
			else if(Temp_buf[3] & 0x4)  //Check to see if thermocouple is short circuited to VCC
			{
				status = TEMP_SCV_FAULT;
			}
			*temp = pTemp;
			memset(Temp_buf,0,5);
		}
		else
		{
			*temp >>= 2;
			*temp *= 0.25; /*Temperature Resolution Calculation*/

			pTemp = *temp;
			memset(Temp_buf,0,5);
		}
	}
	else if(sensor==BOTTOM_PAN)
	{
		HAL_GPIO_WritePin(MAX_SPI4_CS2_GPIO_Port, MAX_SPI4_CS2_Pin, GPIO_PIN_RESET);
		//HAL_Delay(5);

		/*Configure with the appropriate SPI handle structure*/
		HAL_SPI_Receive(&hspi4,Temp_buf,4,100);

		*temp = Temp_buf[0] << 8;

		*temp |= Temp_buf[1];

		HAL_GPIO_WritePin(MAX_SPI4_CS2_GPIO_Port, MAX_SPI4_CS2_Pin, GPIO_PIN_SET);

		if(*temp & 0x1)
		{
			if(Temp_buf[3] & 0x1) //Check to see if thermocouple is open
			{
				status = TEMP_OC_FAULT;
			}
			else if(Temp_buf[3] & 0x2) //Check to see if thermocouple is short circuited to GND
			{
				status = TEMP_SCG_FAULT;
			}
			else if(Temp_buf[3] & 0x4)  //Check to see if thermocouple is short circuited to VCC
			{
				status = TEMP_SCV_FAULT;
			}
			*temp = pTemp;
			memset(Temp_buf,0,5);
		}
		else
		{
			*temp >>= 2;
			*temp *= 0.25; /*Temperature Resolution Calculation*/
			pTemp = *temp;
			memset(Temp_buf,0,5);
		}
	}
	return status; /*Return the Status of the thermocouple*/
}
