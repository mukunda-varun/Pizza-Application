/*
 * i2c_TOF.h
 *
 *  Created on: Apr 27, 2021
 *      Author: Varun
 */

#ifndef INC_I2C_TOF_H_
#define INC_I2C_TOF_H_

#include "main.h"


/**TOF Flour Connector Presence**/
#if	FLOUR_PRESENCE_SENSOR
	extern I2C_HandleTypeDef hi2c2;
	HAL_StatusTypeDef ret;
	uint8_t buf[4] = {0};
	uint16_t distanceValueMM = 0;
	float finalDistanceMM = 0;
	uint16_t distanceValueArray[50] = { 0 };
	float getTOFValue(uint8_t slaveAddress);
#endif

#if FLOUR_PRESENCE_SENSOR
	#define	TOF_FILTER	20
#endif


/**TOF Flour Connector Presence**/

#endif /* INC_I2C_TOF_H_ */
