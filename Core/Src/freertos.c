/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float bat_vol;
const float bat_vol_lim = 7.0f; //[V]
float pressure;
AxesRaw acc, gyro, mag;
AHRS_State ahrs;
MotorInput motor_input;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId controlTaskHandle;
osThreadId logTaskHandle;
osSemaphoreId ControlSemaphoreHandle;
osSemaphoreId LogSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartControlTask(void const *argument);
void StartLogTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ControlSemaphore */
  osSemaphoreDef(ControlSemaphore);
  ControlSemaphoreHandle = osSemaphoreCreate(osSemaphore(ControlSemaphore), 1);

  /* definition and creation of LogSemaphore */
  osSemaphoreDef(LogSemaphore);
  LogSemaphoreHandle = osSemaphoreCreate(osSemaphore(LogSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityNormal, 0, 128);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of logTask */
  osThreadDef(logTask, StartLogTask, osPriorityNormal, 0, 128);
  logTaskHandle = osThreadCreate(osThread(logTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
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

      float duty = 200;
      // HAL_Delay(2000);
      // PWM_Update(&motor1, duty);
      PWM_Update(&motor2, duty);
      // PWM_Update(&motor3, duty);
      PWM_Update(&motor4, duty);
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
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief Function implementing the controlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const *argument)
{
  /* USER CODE BEGIN StartControlTask */
  HAL_TIM_Base_Start_IT(&htim12);
  /* Infinite loop */
  for (;;)
  {
    osSemaphoreWait(ControlSemaphoreHandle, osWaitForever);
    ReadBatteryVoltage(&bat_vol);
    // ReadPressure(&pressure);
    ReadSensor(&acc, &gyro, &mag);
    UpdateAHRS(&acc, &gyro, &mag, &ahrs);
    UpdateControl(&ahrs, &motor_input, bat_vol);

    if (bat_vol > bat_vol_lim)
    {
      Write_GPIO(USER_LED1, 1);
      // DriveMotor(&motor1, &motor2, &motor3, &motor4,
      //            &motor5, &motor6, &motor7, &motor8, &motor_input);
    }
    else
    {
      Write_GPIO(USER_LED1, 0);
      BrakeMotor(&motor1, &motor2, &motor3, &motor4, &motor5, &motor6, &motor7, &motor8);
    }
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
 * @brief Function implementing the logTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLogTask */
void StartLogTask(void const *argument)
{
  /* USER CODE BEGIN StartLogTask */
  HAL_TIM_Base_Start_IT(&htim12);
  /* Infinite loop */
  for (;;)
  {
    osSemaphoreWait(LogSemaphoreHandle, osWaitForever);
    printf("%.3f\r\n", ahrs.euler.y);
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
    // motor_input.inputs[0] = 0.3; // [V]
    // Voltage2Duty(&motor_input, &bat_vol);
    // DriveMotor(&motor1, &motor2, &motor3, &motor4,
    //            &motor5, &motor6, &motor7, &motor8, &motor_input);
    // printf("%.3f, %.3f\n", bat_vol, motor_input.inputs[0]);

    // printf("%f, %f, %f\n", acc.x, acc.y, acc.z);
    // printf("%f, %f, %f\n", gyro.x, gyro.y, gyro.z);
    // printf("%.3f\r\n", gyro.x);
    // printf("%.3f, %.3f, %.3f\n", mag.x, mag.y, mag.z);
    // printf("%.3f\t%.3f\t%.3f\t%.3f\t\r\n", ahrs.q.q0, ahrs.q.q1, ahrs.q.q2, ahrs.q.q3);
    // printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t\r\n", motor_input.inputs[0], motor_input.inputs[1], motor_input.inputs[2], motor_input.inputs[3],
    //        motor_input.inputs[4], motor_input.inputs[5], motor_input.inputs[6], motor_input.inputs[7]);
    // printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t\r\n", coeff_Fprop[0], coeff_Fprop[1], coeff_Fprop[2], coeff_Fprop[3], coeff_Fprop[4], coeff_Fprop[5], coeff_Fprop[6], coeff_Fprop[7]);
    // printf("pressure = %f\n", pressure);
    // printf("battery = %f\n", bat_vol);

    // printf("invert mode = %d\n", inverted_mode);
  }
  /* USER CODE END StartLogTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
