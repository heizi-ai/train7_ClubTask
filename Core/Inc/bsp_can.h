#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

extern CAN_HandleTypeDef hcan1;
extern uint16_t motor_angle;
extern int16_t motor_speed;
extern uint8_t motor_temp;
extern int16_t motor_current;
extern uint32_t last_time;
extern volatile uint32_t motor_flag;

void can_filter_init(void);
void can_tx_header_init(void);
void can_tx_send(int16_t voltage);


#endif
