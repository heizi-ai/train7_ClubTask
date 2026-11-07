#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "tim.h"
#include "freertos_os2.h"
#include "freertos_mpool.h"
#include "cmsis_os2.h"
#include "PID.h"
void Uart_Motor(volatile uint32_t size,char * receiveData,float *motor_target);

#endif //MOTOR_H
