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

arm_matrix_instance_f32 mat_pinvM;
arm_matrix_instance_f32 mat_Tdes;
arm_matrix_instance_f32 mat_Fprop;

float coeff_Tdes[3];
float coeff_Fprop[8];

void InitializeController();
void InitializePID();
void UpdateControl(AHRS_State *ahrs, MotorInput *motor_input, float *bat_vol);
void CalcMotorInput(MotorInput *motor_input, float *bat_vol);
float Voltage2Duty(float voltage, float *bat_vol);

#endif /* __CONTROL_H_ */