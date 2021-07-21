/*
 * MAX_31855.h
 *
 *  Created on: Sep 23, 2020
 *      Author: Madhu
 */

#ifndef INC_MAX_31855_H_
#define INC_MAX_31855_H_
#include "main.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

#define TOP_PAN_CUTOFF    110
#define BOTTOM_PAN_CUTOFF 135
typedef enum{
	TEMP_OK = 0,
	TEMP_SCV_FAULT, /*Thermocouple is short circuited to VCC*/
	TEMP_SCG_FAULT, /*Thermocouple is short circuited to GND*/
	TEMP_OC_FAULT, /*Thermocouple is open*/
}Temp_status;
typedef enum{
	TOP_PAN,
	BOTTOM_PAN
}Sensor_type;
Temp_status ReadTemperature(uint16_t *temp,Sensor_type sensor);
void TempError(Temp_status error);
void Temperature_control(uint16_t Top_Pan_Cutoff,uint16_t Bottom_Pan_Cutoff);
#endif /* INC_MAX_31855_H_ */
