/*
 *  motor.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"

typedef struct
{
    float input1, input2, input3, input4;
    float input5, input6, input7, input8;
} MotorInput;

void DriveMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
                Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8, MotorInput *motor_input);
void BrakeMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
                Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8);

#endif /* __MOTOR_H_ */