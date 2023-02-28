/*
 *  motor.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "motor.h"

void DriveMotor(MotorControl *motor)
{
    if (motor->input1 > MOTOR_MAX_PWM_VALUE)
            motor->input1 = MOTOR_MAX_PWM_VALUE;
        else if (motor->input1 < -MOTOR_MAX_PWM_VALUE)
            motor->input1 = -MOTOR_MAX_PWM_VALUE;

        if (motor->input1 > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE - motor->input1);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE + motor->input1);
        }
}

void BrakeMotor(MotorControl *motor)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
}
