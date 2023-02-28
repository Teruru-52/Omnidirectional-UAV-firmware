/*
 *  control.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "ahrs.h"
#include "quaternion.h"
#include "motor.h"
#include "basic_math.h"

void InitializePID();
void UpdateControl(AHRS_State *ahrs, MotorControl *motor);

#endif /* __CONTROL_H_ */