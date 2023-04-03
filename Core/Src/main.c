/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.h"
#include "lps25hb.h"
#include "speaker.h"
#include "ahrs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// extern osSemaphoreId Control_SemaphoreHandle;
// extern osSemaphoreId AHRS_SemaphoreHandle;
int count = 0;
float bat_vol;
float pressure;
AxesRaw acc, gyro, mag;
AHRS_State ahrs;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM1) // 1kHz interruption
  {
    // osSemaphoreRelease(Control_SemaphoreHandle);
  }

  if (htim->Instance == TIM12) // 100Hz interruption
  {
    // osSemaphoreRelease(AHRS_SemaphoreHandle);
    ReadSensor(&acc, &gyro, &mag);
    // ReadPressure(&pressure);
    // UpdateMadgwickFilter(&acc, &gyro, &mag, &ahrs);
    UpdateMadgwickFilterIMU(&acc, &gyro, &ahrs);
    // UpdateEKF(&acc, &gyro, &mag, &ahrs);
    count = (count + 1) % 100;

    if (count == 99)
    {
      Write_GPIO(USER_LED4, 1);
    }
    else
    {
      Write_GPIO(USER_LED4, 0);
    }

    if (count % 20 == 0)
    {
      // float roll = atan2(2.0f * (ahrs.q.q0 * ahrs.q.q1 + ahrs.q.q2 * ahrs.q.q3), ahrs.q.q0 * ahrs.q.q0 - ahrs.q.q1 * ahrs.q.q1 - ahrs.q.q2 * ahrs.q.q2 + ahrs.q.q3 * ahrs.q.q3);
      // float pitch = asin(2.0f * (ahrs.q.q0 * ahrs.q.q2 - ahrs.q.q1 * ahrs.q.q3));
      // float yaw = atan2(2.0f * (ahrs.q.q1 * ahrs.q.q2 + ahrs.q.q0 * ahrs.q.q3), ahrs.q.q0 * ahrs.q.q0 + ahrs.q.q1 * ahrs.q.q1 - ahrs.q.q2 * ahrs.q.q2 - ahrs.q.q3 * ahrs.q.q3);

      // printf("%f, %f, %f\n", acc.x, acc.y, acc.z);
      printf("%f, %f, %f\n", gyro.x, gyro.y, gyro.z);
      // printf("%3f, %3f, %3f\n", mag.x, mag.y, mag.z);
      // printf("%3f\t%3f\t%3f\t%3f\t\r\n", ahrs.q.q0, ahrs.q.q1, ahrs.q.q2, ahrs.q.q3);
      // printf("%3f\t%3f\t%3f\t\r\n", roll, pitch, yaw);
      // printf("pressure = %f\n", pressure);
    }
  }
  /* USER CODE END Callback 1 */
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

  Write_GPIO(USER_LED2, 1);
  ReadBatteryVoltage(&bat_vol);
  printf("battery = %f\n", bat_vol);
  InitializeIMU();
  Initialize_LPS25HB();
  CalcAccOffset(&acc);
  CalcGyroOffset(&gyro);
  InitializeAHRS(&ahrs);
  Beep();
  Write_GPIO(USER_LED2, 0);

  Motor_TIM_Init();

  Write_GPIO(USER_LED1, 1);
  Write_GPIO(USER_LED3, 1);
  Write_GPIO(USER_LED4, 1);

  int duty = 200;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (Read_GPIO(USER_SW) == 1)
    {
      Write_GPIO(USER_LED3, 0);
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
    else
    {
      Write_GPIO(USER_LED3, 1);
      // HAL_Delay(3000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE);
      // HAL_Delay(2000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE - duty);
      // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MOTOR_MAX_PWM_VALUE);
      // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MOTOR_MAX_PWM_VALUE - duty);
      // HAL_Delay(2000);
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
