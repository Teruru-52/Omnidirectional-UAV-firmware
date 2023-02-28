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
#include "ahrs.h"
#include "control.h"
#include "motor.h"
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
/* USER CODE BEGIN Variables */
AxesRaw acc, gyro, mag;
AHRS_State ahrs;
MotorControl motor;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId AHRS_TaskHandle;
osThreadId Control_TaskHandle;
osThreadId LED_TaskHandle;
osThreadId Speaker_TaskHandle;
osSemaphoreId Control_SemaphoreHandle;
osSemaphoreId AHRS_SemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartAHRS_Task(void const *argument);
void StartControl_Task(void const *argument);
void StartLED_Task(void const *argument);
void StartSpeaker_Task(void const *argument);

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
  /* definition and creation of Control_Semaphore */
  osSemaphoreDef(Control_Semaphore);
  Control_SemaphoreHandle = osSemaphoreCreate(osSemaphore(Control_Semaphore), 1);

  /* definition and creation of AHRS_Semaphore */
  osSemaphoreDef(AHRS_Semaphore);
  AHRS_SemaphoreHandle = osSemaphoreCreate(osSemaphore(AHRS_Semaphore), 1);

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

  /* definition and creation of AHRS_Task */
  osThreadDef(AHRS_Task, StartAHRS_Task, osPriorityIdle, 0, 128);
  AHRS_TaskHandle = osThreadCreate(osThread(AHRS_Task), NULL);

  /* definition and creation of Control_Task */
  osThreadDef(Control_Task, StartControl_Task, osPriorityIdle, 0, 128);
  Control_TaskHandle = osThreadCreate(osThread(Control_Task), NULL);

  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, StartLED_Task, osPriorityIdle, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of Speaker_Task */
  osThreadDef(Speaker_Task, StartSpeaker_Task, osPriorityIdle, 0, 128);
  Speaker_TaskHandle = osThreadCreate(osThread(Speaker_Task), NULL);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAHRS_Task */
/**
 * @brief Function implementing the AHRS_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAHRS_Task */
void StartAHRS_Task(void const *argument)
{
  /* USER CODE BEGIN StartAHRS_Task */
  /* Infinite loop */
  for (;;)
  {
    osSemaphoreWait(AHRS_SemaphoreHandle, osWaitForever);
    ReadSensor(&acc, &gyro, &mag);
    UpdateEKF(&acc, &gyro, &mag, &ahrs);
  }
  /* USER CODE END StartAHRS_Task */
}

/* USER CODE BEGIN Header_StartControl_Task */
/**
 * @brief Function implementing the Control_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControl_Task */
void StartControl_Task(void const *argument)
{
  /* USER CODE BEGIN StartControl_Task */
  /* Infinite loop */
  for (;;)
  {
    osSemaphoreWait(Control_SemaphoreHandle, osWaitForever);
    UpdateControl(&ahrs, &motor);
    DriveMotor(&motor);
  }
  /* USER CODE END StartControl_Task */
}

/* USER CODE BEGIN Header_StartLED_Task */
/**
 * @brief Function implementing the LED_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLED_Task */
void StartLED_Task(void const *argument)
{
  /* USER CODE BEGIN StartLED_Task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED_Task */
}

/* USER CODE BEGIN Header_StartSpeaker_Task */
/**
 * @brief Function implementing the Speaker_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSpeaker_Task */
void StartSpeaker_Task(void const *argument)
{
  /* USER CODE BEGIN StartSpeaker_Task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSpeaker_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
