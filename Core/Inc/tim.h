/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.h
 * @brief   This file contains all the function prototypes for
 *          the tim.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim12;

/* USER CODE BEGIN Private defines */
  #define MOTOR_MAX_PWM_VALUE 999.0f
#define MOTOR_UPPER_PWM_VALUE 270.0f
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_TIM9_Init(void);
void MX_TIM12_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
  typedef struct _Motor
  {
    FunctionalState OutputPWM;
    TIM_HandleTypeDef *htim;
    uint32_t CHANNEL_1;
    uint32_t CHANNEL_2;
  } Motor;

  void Base_TIM_Init(void);
  void Motor_TIM_Init(void);
  void Speaker_TIM_Init(void);
  void PWM_Start(Motor *motor);
  void PWM_Stop(Motor *motor);
  void PWM_Set(Motor *motor, float duty);
  void PWM_Update(Motor *motor, float duty);

  extern Motor motor1;
  extern Motor motor2;
  extern Motor motor3;
  extern Motor motor4;
  extern Motor motor5;
  extern Motor motor6;
  extern Motor motor7;
  extern Motor motor8;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

