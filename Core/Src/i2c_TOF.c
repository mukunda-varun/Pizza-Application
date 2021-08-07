/*
 * i2c_TOF.c
 *
 *  Created on: Apr 27, 2021
 *      Author: Varun
 */
#include "i2c_TOF.h"

#if FLOUR_PRESENCE_SENSOR
	float getTOFValue(uint8_t slaveAddress)
	{
		distanceValueMM=0;
		finalDistanceMM=0;
		for(int count=0;count < TOF_FILTER; count++)
		{
			ret = HAL_I2C_Init(&hi2c2);
			osDelay(10);
			ret = HAL_I2C_Master_Transmit(&hi2c2, slaveAddress, 0x00, 1, HAL_MAX_DELAY);
			osDelay(10);
			ret = HAL_I2C_Master_Receive(&hi2c2, slaveAddress, buf, 2, HAL_MAX_DELAY);
			osDelay(10);
			ret = HAL_I2C_DeInit(&hi2c2);
			distanceValueArray[count]=(buf[0]<<8)|buf[1];
			distanceValueMM+=distanceValueArray[count];
			distanceValueArray[count]=0;
		}

		finalDistanceMM = distanceValueMM/TOF_FILTER;
		return finalDistanceMM;
	}
#endif

