/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app_tasks.h"
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
/* USER CODE END Variables */
/* Definitions for LED_TASK */
osThreadId_t LED_TASKHandle;
const osThreadAttr_t LED_TASK_attributes = {
  .name = "LED_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KEY_TASK */
osThreadId_t KEY_TASKHandle;
const osThreadAttr_t KEY_TASK_attributes = {
  .name = "KEY_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SERVO_TASK */
osThreadId_t SERVO_TASKHandle;
const osThreadAttr_t SERVO_TASK_attributes = {
  .name = "SERVO_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PRINT_SPEED_TAS */
osThreadId_t PRINT_SPEED_TASHandle;
const osThreadAttr_t PRINT_SPEED_TAS_attributes = {
  .name = "PRINT_SPEED_TAS",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_TASK */
osThreadId_t UART_TASKHandle;
const osThreadAttr_t UART_TASK_attributes = {
  .name = "UART_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for KEY_SERVO */
osMessageQueueId_t KEY_SERVOHandle;
const osMessageQueueAttr_t KEY_SERVO_attributes = {
  .name = "KEY_SERVO"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void START_LED_TASK(void *argument);
void START_KEY_TASK(void *argument);
void START_SERVO_TASK(void *argument);
void START_PRINT_SPEED_TASK(void *argument);
void START_UART_TASK(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of KEY_SERVO */
  KEY_SERVOHandle = osMessageQueueNew (16, sizeof(uint32_t), &KEY_SERVO_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LED_TASK */
  LED_TASKHandle = osThreadNew(START_LED_TASK, NULL, &LED_TASK_attributes);

  /* creation of KEY_TASK */
  KEY_TASKHandle = osThreadNew(START_KEY_TASK, NULL, &KEY_TASK_attributes);

  /* creation of SERVO_TASK */
  SERVO_TASKHandle = osThreadNew(START_SERVO_TASK, NULL, &SERVO_TASK_attributes);

  /* creation of PRINT_SPEED_TAS */
  PRINT_SPEED_TASHandle = osThreadNew(START_PRINT_SPEED_TASK, NULL, &PRINT_SPEED_TAS_attributes);

  /* creation of UART_TASK */
  UART_TASKHandle = osThreadNew(START_UART_TASK, NULL, &UART_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_START_LED_TASK */
/**
  * @brief  Function implementing the LED_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_START_LED_TASK */
void START_LED_TASK(void *argument)
{
  /* USER CODE BEGIN START_LED_TASK */
  APP_LED_TASK(argument);
  /* USER CODE END START_LED_TASK */
}

/* USER CODE BEGIN Header_START_KEY_TASK */
/**
* @brief Function implementing the KEY_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_START_KEY_TASK */
void START_KEY_TASK(void *argument)
{
  /* USER CODE BEGIN START_KEY_TASK */
  APP_KEY_TASK(argument);
  /* USER CODE END START_KEY_TASK */
}

/* USER CODE BEGIN Header_START_SERVO_TASK */
/**
* @brief Function implementing the SERVO_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_START_SERVO_TASK */
void START_SERVO_TASK(void *argument)
{
  /* USER CODE BEGIN START_SERVO_TASK */
  APP_SERVO_TASK(argument);
  /* USER CODE END START_SERVO_TASK */
}

/* USER CODE BEGIN Header_START_PRINT_SPEED_TASK */
/**
* @brief Function implementing the PRINT_SPEED_TAS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_START_PRINT_SPEED_TASK */
void START_PRINT_SPEED_TASK(void *argument)
{
  /* USER CODE BEGIN START_PRINT_SPEED_TASK */
  APP_PRINT_SPEED_TASK(argument);
  /* USER CODE END START_PRINT_SPEED_TASK */
}

/* USER CODE BEGIN Header_START_UART_TASK */
/**
* @brief Function implementing the UART_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_START_UART_TASK */
void START_UART_TASK(void *argument) {
  /* USER CODE BEGIN START_UART_TASK */
  APP_UART_TASK(argument);
  /* USER CODE END START_UART_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

