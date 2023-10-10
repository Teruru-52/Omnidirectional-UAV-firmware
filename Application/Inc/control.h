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
#include "solver.h"

arm_matrix_instance_f32 mat_pinvM;
arm_matrix_instance_f32 mat_Tdes;
arm_matrix_instance_f32 mat_Fprop;
arm_matrix_instance_f32 mat_K_vertex;
arm_matrix_instance_f32 mat_x;

float coeff_Tdes[3];
float coeff_Fprop[8];

void InitializeController();
void InitializePID();
void UpdateControl(AHRS_State *ahrs, MotorInput *motor_input, float bat_vol);
// void TestControl(AHRS_State *ahrs);
void UpdateQuaternionControl(AHRS_State *ahrs, MotorInput *motor_input, float bat_vol);
void UpdateEulerControl(AHRS_State *ahrs, MotorInput *motor_input, float bat_vol);
void CalcInputTorqueQuaternion(AHRS_State *ahrs, Quaternion q_des);
void CalcInputTorqueEuler(AHRS_State *ahrs, AxesRaw *error_angle);
void CalcMotorInput(MotorInput *motor_input, float bat_vol);
void CalcInputVelocity(MotorInput *motor_input);
void CalcInputVoltage(MotorInput *motor_input);
void Voltage2Duty(MotorInput *motor_input, float bat_vol);

extern int inverted_mode;

#endif /* __CONTROL_H_ */