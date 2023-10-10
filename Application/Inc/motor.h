/*
 *  motor.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __MOTOR_H_
#define __MOTOR_H_

#define MOTOR_NUM 8

#include "main.h"
#include "speaker.h"

typedef struct
{
    float voltage[8];
    float velocity[8];
    float duty[8];
} MotorInput;

void DriveMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
                Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8, MotorInput *motor_input);
void BrakeMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
                Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8);
void TestMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
               Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8);

#endif /* __MOTOR_H_ */