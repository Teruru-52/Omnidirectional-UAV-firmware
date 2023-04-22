/*
 *  motor.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "motor.h"

void DriveMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
                Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8, MotorInput *motor_input)
{
    PWM_Update(motor1, motor_input->inputs[0]);
    PWM_Update(motor2, motor_input->inputs[1]);
    PWM_Update(motor3, motor_input->inputs[2]);
    PWM_Update(motor4, motor_input->inputs[3]);
    PWM_Update(motor5, motor_input->inputs[4]);
    PWM_Update(motor6, motor_input->inputs[5]);
    PWM_Update(motor7, motor_input->inputs[6]);
    PWM_Update(motor8, motor_input->inputs[7]);
}

void BrakeMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
                Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8)
{
    PWM_Stop(motor1);
    PWM_Stop(motor2);
    PWM_Stop(motor3);
    PWM_Stop(motor4);
    PWM_Stop(motor5);
    PWM_Stop(motor6);
    PWM_Stop(motor7);
    PWM_Stop(motor8);
}

void TestMotor(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4,
               Motor *motor5, Motor *motor6, Motor *motor7, Motor *motor8)
{
    float duty = 50;
    int dt = 200;
    PWM_Update(motor1, duty);
    HAL_Delay(dt);
    PWM_Stop(motor1);
    Beep();
    PWM_Update(motor2, duty);
    HAL_Delay(dt);
    PWM_Stop(motor2);
    Beep();
    PWM_Update(motor3, duty);
    HAL_Delay(dt);
    PWM_Stop(motor3);
    Beep();
    PWM_Update(motor4, duty);
    HAL_Delay(dt);
    PWM_Stop(motor4);
    Beep();
    PWM_Update(motor5, duty);
    HAL_Delay(dt);
    PWM_Stop(motor5);
    Beep();
    PWM_Update(motor6, duty);
    HAL_Delay(dt);
    PWM_Stop(motor6);
    Beep();
    PWM_Update(motor7, duty);
    HAL_Delay(dt);
    PWM_Stop(motor7);
    Beep();
    PWM_Update(motor8, duty);
    HAL_Delay(dt);
    PWM_Stop(motor8);
    Beep();
}
