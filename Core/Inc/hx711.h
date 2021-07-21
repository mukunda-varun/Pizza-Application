
/*
 * hx711.h
 *
 *  Created on: 23/09/20
 *      Author: madhu
 */

#ifndef HX711_H_
#define HX711_H_

#include "main.h"
#include "cmsis_os.h"

typedef struct
{
	GPIO_TypeDef* PD_SCK_PinType;	//GPIOx
	uint16_t PD_SCK_PinNumber;		//GPIO_Pin
	GPIO_TypeDef* DOUT_PinType;		//GPIOx
	uint16_t DOUT_PinNumber;		//GPIO_Pin
	uint32_t offset;
	float SCALE;
	uint8_t gain;	// 0 Input channel A, gain=128
					// 1 Input channel B, gain=32
					// 2 Input channel A, gain=64
} HX711;

static const uint8_t LOW      = 0; // digital low
static const uint8_t HIGH     = 1; // digital high



unsigned long HX711_Read(HX711* H);
uint32_t HX711_AvgRead(HX711* H, int times);
uint32_t HX711_Tare(HX711* H, int times);
uint32_t HX711_GetValue(HX711* H);
uint32_t HX711_GetAvgValue(HX711* H, int times);
float HX711_Get_units(int times);
uint8_t Hx711_shiftInMsbFirst(HX711* H);
uint32_t Hx711_readRaw(HX711* H);
void delay_us (uint32_t us);
#endif /* HX711_H_ */

