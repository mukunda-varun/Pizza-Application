/*
 * hx711.c
 *
 *
 *
 */

#include "hx711.h"
#include "main.h"
#include "cmsis_os.h"
extern TIM_HandleTypeDef htim17;
void delay_us (uint32_t us)
{
//	__HAL_TIM_SET_COUNTER(&htim17,0);  // set the counter value a 0
//	while (__HAL_TIM_GET_COUNTER(&htim17) < us);  // wait for the counter to reach the us input in the parameter
}
/* @Brief: This functiton shifts the read 14-bit value from HX711 to 8bit data to convert into actual weight
 * */
uint8_t Hx711_shiftInMsbFirst(HX711* H)
{
 uint8_t value = 0;
 for (uint8_t i = 0; i < 8; ++i)
 {
  HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber, GPIO_PIN_SET);
//  delay_us(10);
  value |= HAL_GPIO_ReadPin(H->DOUT_PinType, H->DOUT_PinNumber) << (7 - i);
  HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber,GPIO_PIN_RESET);
//  delay_us(10);
 }
  return value;
}

unsigned long HX711_Read(HX711* H)  //
{
 unsigned long val = 0;

_Bool isNegative = 0;


 HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber, GPIO_PIN_SET);
 HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber, GPIO_PIN_RESET);


 while (HAL_GPIO_ReadPin(H->DOUT_PinType, H->DOUT_PinNumber))
 {

 }

 for (uint8_t i = 0; i < 24; i++)
 {
  HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber, GPIO_PIN_SET);
  val = val << 1;
  HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber,GPIO_PIN_RESET);

  if (HAL_GPIO_ReadPin(H->DOUT_PinType, H->DOUT_PinNumber))
  {
   if (i == 0)
   {
   isNegative = 1;
   }
  val++;
  }

  if (isNegative)
  {
   val = val ^ 0xFF000000;
  }

  HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber, GPIO_PIN_SET);
  HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber,GPIO_PIN_RESET);
  for (uint8_t i = 0; i < H->gain; i++)
  {
  		HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber,GPIO_PIN_RESET);
  	}
 }
 return val;
}


uint32_t HX711_AvgRead(HX711* H, int times)
{
	int sum = 0;
	for (uint8_t i = 0; i < times; i++) {
		sum += HX711_Read(H);
	}
	return sum / times;
}

uint32_t HX711_Tare(HX711* H, int times)
{
	uint32_t Offset;
	Offset= HX711_AvgRead(H, times);
	return Offset;
}

uint32_t HX711_GetValue(HX711* H)
{
	return HX711_Read(H)-(H->offset);
}

uint32_t HX711_GetAvgValue(HX711* H, int times)
{
	return HX711_AvgRead(H, times)-(H->offset);
}

void HX711_SetScale(float scale)
{
 //float SCALE = scale;
}


uint32_t Hx711_readRaw(HX711* H)
{

    uint32_t value = 0;
    uint8_t data[3] = { 0 };
    uint8_t filler = 0x00;
    while (HAL_GPIO_ReadPin(H->DOUT_PinType, H->DOUT_PinNumber));
    // pulse the clock pin 24 times to read the data
    data[2] = Hx711_shiftInMsbFirst(H);
    data[1] = Hx711_shiftInMsbFirst(H);
    data[0] = Hx711_shiftInMsbFirst(H);

    // set the channel and the gain factor for the next reading using the clock pin
    for (unsigned int i = 0; i < H->gain; i++)
    {
     HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber, GPIO_PIN_SET);
     HAL_GPIO_WritePin(H->PD_SCK_PinType, H->PD_SCK_PinNumber,GPIO_PIN_RESET);
    }

// Datasheet indicates the value is returned as a two's complement value
// Flip all the bits
//    data[2] = ~data[2];
//    data[1] = ~data[1];
//    data[0] = ~data[0];

    // Replicate the most significant bit to pad out a 32-bit signed integer
    if ( data[2] & 0x80 )
    {
        filler = 0xFF;
    }
    else
    {
        filler = 0x00;
    }

    // Construct a 32-bit signed integer
    value = ( (uint32_t)(filler)  << 24
            | (uint32_t)(data[2]) << 16
            | (uint32_t)(data[1]) << 8
            | (uint32_t)(data[0]) );
    value=value/H->SCALE;
    return value;
}

