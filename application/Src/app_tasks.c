/**
 * @file    app_tasks.c
 * @brief   应用层 —— 所有任务主体 + 串口协议解析
 *
 * 分层说明：
 *   Core/Src/freertos.c 由 CubeMX 生成，里面只保留 START_xxx_TASK 这一层"壳子"，
 *   真正跑循环的代码全部集中在这里，业务逻辑与生成代码彻底分离。
 */

#include "app_tasks.h"
#include "cmsis_os2.h"
#include "main.h"
#include "bsp_can.h"
#include "motor.h"
#include "servo.h"
#include "led_state.h"
#include "usart.h"
#include "PID.h"
#include "stdio.h"
#include "string.h"

/* KEY_TASK 与 SERVO_TASK 之间传消息用的队列，句柄由 CubeMX 在 freertos.c 里定义 */
extern osMessageQueueId_t KEY_SERVOHandle;

/* ---------------- 滤波系数 ---------------- */
#define PID_FILTER_OLD_WEIGHT 0.7
#define PID_FILTER_NEW_WEIGHT 0.3
#define SPEED_FILTER_OLD_WEIGHT 0.7
#define SPEED_FILTER_NEW_WEIGHT 0.3

/* ---------------- 任务间共享的数据 ---------------- */
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

/* ---------------- 任务主体 ---------------- */

/* LED 任务：每 10ms 刷新一次状态灯 */
void APP_LED_TASK(void *argument) {
    for (;;) {
        led_state();
        osDelay(10);
    }
}

/* 按键任务：消抖 → 入队一次 → 等松开（保证按一次只算一次） */
void APP_KEY_TASK(void *argument) {
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
  
}

/* 舵机任务：阻塞等待按键消息，收到一次就转 20° */
void APP_SERVO_TASK(void *argument) {
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
  
}

/* 打印任务：每 2 秒把当前转速通过串口发出去 */
void APP_PRINT_SPEED_TASK(void *argument) {
  /* Infinite loop */
  for(;;)
  {

    /* 每 2 秒通过串口打印一次当前转速 */
    sprintf(sendSpeedData,"%frpm\r\n",motor_dealspeed);
    HAL_UART_Transmit_DMA(&huart6,sendSpeedData,strlen(sendSpeedData));
    osDelay(2000);
  }
  
}

/* 控制任务：滤波 → 解析串口指令 → PID → 输出平滑 → 发 CAN（10ms 周期） */
void APP_UART_TASK(void *argument) {
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
  
}

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
