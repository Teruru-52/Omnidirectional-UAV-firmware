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
    if (motor_input->input1 > MOTOR_MAX_PWM_VALUE)
        motor_input->input1 = MOTOR_MAX_PWM_VALUE;
    else if (motor_input->input1 < -MOTOR_MAX_PWM_VALUE)
        motor_input->input1 = -MOTOR_MAX_PWM_VALUE;

    PWM_Update(motor1, motor_input->input1);
    PWM_Update(motor2, motor_input->input2);
    PWM_Update(motor3, motor_input->input3);
    PWM_Update(motor4, motor_input->input4);
    PWM_Update(motor5, motor_input->input5);
    PWM_Update(motor6, motor_input->input6);
    PWM_Update(motor7, motor_input->input7);
    PWM_Update(motor8, motor_input->input8);
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
