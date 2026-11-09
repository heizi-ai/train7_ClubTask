#ifndef APP_TASKS_H
#define APP_TASKS_H

/* 各任务主体：由 CubeMX 生成的 START_xxx_TASK 壳子调用 */
void APP_LED_TASK(void *argument);
void APP_KEY_TASK(void *argument);
void APP_SERVO_TASK(void *argument);
void APP_PRINT_SPEED_TASK(void *argument);
void APP_UART_TASK(void *argument);

#endif //APP_TASKS_H
