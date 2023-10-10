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
#include "dma.h"
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
#include "control.h"
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
float bat_vol;
const float bat_vol_lim = 7.5f; //[V]
float pressure;
AxesRaw acc, gyro, mag;
AHRS_State ahrs;
MotorInput motor_input;

int count = 0;
extern float bat_vol;
// extern float pressure;
extern AxesRaw acc, gyro, mag;
extern AHRS_State ahrs;

void OutputLog()
{
  // printf("%.3f\r\n", ahrs.euler.y);
  // printf("%.3f\r\n", -atan2(acc.y, -acc.z));
  // printf("%.3f\t%.3f\t%.3f\r\n", ahrs.euler.x, ahrs.euler.y, ahrs.euler.z);

  // float roll = -atan2(acc.y, -acc.z);
  // float pitch = atan2(acc.x, Sqrt(acc.y * acc.y + acc.z * acc.z));
  // float yaw = atan2((-mag.y * cos(roll) + mag.z * sin(roll)), (mag.x * cos(pitch) + mag.y * sin(roll) * sin(pitch) + mag.z * cos(roll) * sin(pitch)));
  // printf("%.3f\r\n", yaw);

  // for magnetometer calibration
  // ReadRawMag(&mag);
  // printf("%.3f, %.3f, %.3f\n", mag.x, mag.y, mag.z);

  // for 2-point suspention
  // printf("%f\n", gyro.x);

  // for measurement of the rotation velocity of propeller
  // about 2.27 [V] max
  // motor_input.voltage[0] = 0.3; // [V]
  // Voltage2Duty(&motor_input, bat_vol);
  // DriveMotor(&motor1, &motor2, &motor3, &motor4,
  //            &motor5, &motor6, &motor7, &motor8, &motor_input);
  // printf("%.3f, %.3f\n", bat_vol, motor_input.duty[0]);

  // printf("%f, %f, %f\n", acc.x, acc.y, acc.z);
  // printf("%f, %f, %f\n", gyro.x, gyro.y, gyro.z);
  // printf("%.3f\r\n", gyro.x);
  // printf("%.3f, %.3f, %.3f\n", mag.x, mag.y, mag.z);
  // printf("%.3f\t%.3f\t%.3f\t%.3f\t\r\n", ahrs.q.q0, ahrs.q.q1, ahrs.q.q2, ahrs.q.q3);
  // printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t\r\n", motor_input.voltage[0], motor_input.voltage[1], motor_input.voltage[2], motor_input.voltage[3],
  //        motor_input.voltage[4], motor_input.voltage[5], motor_input.voltage[6], motor_input.voltage[7]);
  // printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t\r\n", coeff_Fprop[0], coeff_Fprop[1], coeff_Fprop[2], coeff_Fprop[3], coeff_Fprop[4], coeff_Fprop[5], coeff_Fprop[6], coeff_Fprop[7]);
  // printf("pressure = %f\n", pressure);
  // printf("battery = %f\n", bat_vol);

  // printf("invert mode = %d\n", inverted_mode);

  // motor_input.velocity[0] = 100.0; // [rps]
  // CalcInputVoltage(&motor_input);
  // motor_input.voltage[0] = 0.5; // [V]
  // Voltage2Duty(&motor_input, bat_vol);
  // printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", motor_input.voltage[0], motor_input.voltage[1], motor_input.voltage[2], motor_input.voltage[3],
  //        motor_input.voltage[4], motor_input.voltage[5], motor_input.voltage[6], motor_input.voltage[7]);
  // printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", motor_input.duty[0], motor_input.duty[1], motor_input.duty[2], motor_input.duty[3],
  //        motor_input.duty[4], motor_input.duty[5], motor_input.duty[6], motor_input.duty[7]);

  printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f,%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",
         -M_PI / 4.0f - ahrs.euler.y, ahrs.gy, err_pitch_sum, coeff_Tdes[1], bat_vol, motor_input.velocity[0], motor_input.velocity[1], motor_input.velocity[2], motor_input.velocity[3], motor_input.velocity[4], motor_input.velocity[5], motor_input.velocity[6], motor_input.velocity[7]);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM1) // 20Hz interruption
  {
    // UpdateQuaternionControl(&ahrs, &motor_input, &bat_vol);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM12) // 100Hz interruption
  {
    ReadBatteryVoltage(&bat_vol);
    // ReadPressure(&pressure);
    ReadSensor(&acc, &gyro, &mag);
    UpdateAHRS(&acc, &gyro, &mag, &ahrs);

    UpdateControl(&ahrs, &motor_input, bat_vol);

    if (bat_vol > bat_vol_lim)
    {
      Write_GPIO(USER_LED1, 1);
      DriveMotor(&motor1, &motor2, &motor3, &motor4,
                 &motor5, &motor6, &motor7, &motor8, &motor_input);
    }
    else
    {
      Write_GPIO(USER_LED1, 0);
      BrakeMotor(&motor1, &motor2, &motor3, &motor4, &motor5, &motor6, &motor7, &motor8);
    }

    count = (count + 1) % 100;

    if (count == 0) // 1Hz
    {
      Write_GPIO(USER_LED4, 1);
    }
    else
    {
      Write_GPIO(USER_LED4, 0);
    }

    if (count % 20 == 0)
    {
      OutputLog();
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
  MX_DMA_Init();
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
  Write_GPIO(USER_LED1, 1);
  Motor_TIM_Init();
  Speaker_TIM_Init();

  // start initialization
  Write_GPIO(USER_LED2, 1);
  ReadBatteryVoltage(&bat_vol);
  printf("battery = %f\n", bat_vol);
  InitializeIMU();
  Initialize_LPS25HB();
  CalcAccOffset(&acc);
  CalcGyroOffset(&gyro);
  SetInitialMag(&mag);
  InitializeAHRS(&ahrs);
  InitializeController();
  Write_GPIO(USER_LED2, 0);
  Write_GPIO(USER_LED3, 1);
  TestMotor(&motor1, &motor2, &motor3, &motor4, &motor5, &motor6, &motor7, &motor8);
  Write_GPIO(USER_LED3, 0);

  // float duty = 200;
  // TestMatrix();
  // for (int i = 0; i < 1000; i++)
  // {
  //   UpdateMadgwickFilterIMU(&acc, &gyro, &ahrs);
  //   printf("%.3f\t%.3f\t%.3f\t%.3f\t\r\n", ahrs.q.q0, ahrs.q.q1, ahrs.q.q2, ahrs.q.q3);
  // }

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim12);
  HAL_UART_Receive_DMA(&huart1, RdBuff, RCV_BUFF_SIZE);
  // inverted_mode = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (Read_GPIO(USER_SW) == 1)
    {
      // BrakeMotor(&motor1, &motor2, &motor3, &motor4, &motor5, &motor6, &motor7, &motor8);
    }
    else
    {
      if (inverted_mode == 0)
      {
        inverted_mode = 1;
        Write_GPIO(USER_LED3, 1);
      }
      else if (inverted_mode == 1)
      {
        inverted_mode = 0;
        Write_GPIO(USER_LED3, 0);
      }

      // float duty = 200;
      // HAL_Delay(2000);
      // PWM_Update(&motor1, duty);
      // PWM_Update(&motor2, duty);
      // PWM_Update(&motor3, duty);
      // PWM_Update(&motor4, duty);
      // PWM_Update(&motor5, duty);
      // PWM_Update(&motor6, duty);
      // PWM_Update(&motor7, duty);
      // PWM_Update(&motor8, duty);
      // HAL_Delay(2000);
    }

    // if (rdUart(&rdData) == TRUE)
    // {
    //   SdBuff[rcvLength++] = rdData;
    //   if ((rdData == CHAR_LF) || (rcvLength >= SND_BUFF_SIZE))
    //   {
    //     HAL_UART_Transmit(&huart1, SdBuff, rcvLength, 0xFFFF);
    //     if (Compare_Stop(SdBuff) == TRUE)
    //     {
    //       Write_GPIO(USER_LED2, 1);
    //     }
    //     else
    //     {
    //       Write_GPIO(USER_LED2, 0);
    //     }
    //     rcvLength = 0;
    //   }
    // }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
