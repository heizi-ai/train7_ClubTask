#include "servo.h"
#include "stdint.h"
#include "tim.h"

Servo_State servo_state=servo_forward;
uint32_t servo_turn_num=0;
uint32_t value=0;

void servo_init(void) {
    servo_turn_num=0;
    value=20000*0.025;
}

void servo_turn(void) {
    if (servo_state==servo_forward) {
        value += 20.0/SERVO_ANGLE*20000*0.1;
        servo_turn_num++;
    }else if (servo_state==servo_backward) {
        value -= 20.0/SERVO_ANGLE*20000*0.1;
        servo_turn_num++;
    }
    if (servo_turn_num+1>SERVO_TURN_NUM&&servo_state==servo_forward) {
        servo_state=servo_backward;
        servo_turn_num=0;
    }else if (servo_turn_num+1>SERVO_TURN_NUM&&servo_state==servo_backward) {
        servo_state=servo_forward;
        servo_turn_num=0;
    }
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,value);
}