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
#include "bsp_can.h"
#include "tim.h"
#include "servo.h"
#include "usart.h"
#include "led_state.h"
#include "PID.h"
#include "stdio.h"
#include "motor.h"

#include "PID.h"
#include "app_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_FILTER_OLD_WEIGHT 0.7
#define PID_FILTER_NEW_WEIGHT 0.3
#define SPEED_FILTER_OLD_WEIGHT 0.7
#define SPEED_FILTER_NEW_WEIGHT 0.3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* PID 输出的一阶低通滤波值：让发出去的电压指令平滑，抑制量化引起的抖动 */
static float filitered_pid_output = 0;
/* 调试用：记录任务剩余栈空间（当前未使用） */
volatile uint32_t print_stack_free=0;
/* 滤波后的转速反馈（一阶低通），PID 用的是它，不是原始 motor_speed */
float motor_dealspeed=0;
/* 串口 DMA 接收缓冲区，存放上位机指令 "motor±N\r\n" */
char receiveData[32]={0};
/* 串口发送缓冲区，用于每 2 秒打印一次转速 */
char sendSpeedData[50]={0};
/* 本帧串口数据的有效字节数，由串口空闲中断写入 */
volatile uint32_t size=0;
/* 目标转速（rpm）：串口解析后写入，PID 读取 */
volatile float motor_target;
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
  /* Infinite loop */
  /* 任务壳子只做转发，真正的循环写在 Application/Src/app_tasks.c，CubeMX 重生成也不会丢 */
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
  /* Infinite loop */
  for(;;)
  {
    /* 按键消抖：连续两次读到低电平才认为真按下 */
    uint32_t msg=1;
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_RESET) {
      osDelay(10);
      if (HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_RESET) {
        osMessageQueuePut(KEY_SERVOHandle, &msg, 0, osWaitForever);
        while (HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_RESET) {
          osDelay(10);
        }
      }
    }
    osDelay(10);
  }
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
  /* 舵机初始化只在上电时做一次；放在循环里会把步数清零，导致永远单方向转 */
  servo_init();
  /* Infinite loop */
  for(;;)
  {
    uint32_t msg=0;
    while (osMessageQueueGet(KEY_SERVOHandle,&msg,0,osWaitForever)==osOK) {
      if (msg==1) {
        servo_turn();
      }
    }
    osDelay(10);
  }
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
  /* Infinite loop */
  for(;;)
  {

    /* 每 2 秒通过串口打印一次当前转速 */
    sprintf(sendSpeedData,"%frpm\r\n",motor_dealspeed);
    HAL_UART_Transmit_DMA(&huart6,sendSpeedData,strlen(sendSpeedData));
    osDelay(2000);
  }
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, receiveData, sizeof(receiveData));

  /* Infinite loop */
  for (;;) {
    // print_stack_free=uxTaskGetStackHighWaterMark(NULL)*sizeof(StackType_t);//之前测试用的，忘删了
    /* 反馈一阶低通：新采样占 SPEED_FILTER_NEW_WEIGHT，旧值占其余部分 */
    motor_dealspeed = motor_speed * SPEED_FILTER_NEW_WEIGHT + motor_dealspeed * SPEED_FILTER_OLD_WEIGHT;
    /* 解析串口指令，更新 motor_target（没收到新指令时内部不做事） */
    Uart_Motor_SetTarget(size, receiveData, &motor_target);
    /* 速度环 PID：输入目标转速与滤波后的实测转速，输出为电压 mV */
    float pid_output = PID_Calculation(&speed_pid, motor_target, motor_dealspeed);
    /* 输出一阶低通：避免电压指令突变，减小电机抖动 */
    filitered_pid_output = filitered_pid_output * PID_FILTER_OLD_WEIGHT + pid_output * PID_FILTER_NEW_WEIGHT;
    /* 把电压指令发给 1 号 GM6020 */
    can_set_motor1_voltage((int16_t) filitered_pid_output);
    // can_set_motor1_voltage(10000);//闭环测试
    osDelay(10);
  }
  /* USER CODE END START_UART_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 串口 DMA 空闲中断：一帧接收完成后进入此回调 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart6) {
    /* 先记录本帧长度，供任务里解析使用 */
    size = Size;
    /* 校验帧头必须是 "motor" 加上 +/- 符号，避免误触发 */
    if (receiveData[0] == 'm' && receiveData[1] == 'o' && receiveData[2] == 't' && receiveData[3] == 'o' && receiveData[
          4] == 'r' && (receiveData[5] == '+' || receiveData[5] == '-')) {
      for (int i = 6; i < Size; i++) {
        if (receiveData[i] >= '0' && receiveData[i] <= '9') {
          motor_flag = 1;
        }
      }
    }
    osDelay(10);
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, receiveData, sizeof(receiveData));
}
/* USER CODE END Application */

