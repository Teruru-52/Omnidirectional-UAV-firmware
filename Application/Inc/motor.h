/*
 *  motor.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"

#define MOTOR_MAX_PWM_VALUE 4999.0f

typedef struct
{
    float input1, input2, input3, input4;
    float input5, input6, input7, input8;
} MotorControl;

void DriveMotor(MotorControl *motor);
void BrakeMotor(MotorControl *motor);

#endif /* __MOTOR_H_ */