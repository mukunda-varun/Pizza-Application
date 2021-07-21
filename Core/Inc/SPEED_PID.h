/*
 * Speed_PID.h
 *
 *  Created on: 27-Jul-2020
 *      Author: svaru
 */

#ifndef INC_SPEED_PID_H_
#define INC_SPEED_PID_H_
#include "main.h"
//#include <math.h>

#define 	Kp_const 	0.1
#define 	Ki_const 	0.1
#define 	Kd_const 	0.0025
#define 	PWM_MAX		100.0
#define 	PWM_MIN		0.0

float PIDcalc(float setpoint,float actual_position,float Kp, float Ki, float Kd);


#endif /* INC_SPEED_PID_H_ */
