#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#define MOTOR1_STDID 0x205
#define MOTOR_CONTROL_STDID 0x1FF
extern CAN_HandleTypeDef hcan1;
extern volatile uint16_t motor_angle;
extern volatile int16_t motor_speed;
extern volatile uint8_t motor_temp;
extern volatile int16_t motor_current;
extern volatile uint32_t last_time;
extern volatile uint32_t motor_flag;

void can_filter_init(void);

void can_tx_header_init(void);

void can_set_motor1_voltage(int16_t voltage);


#endif
