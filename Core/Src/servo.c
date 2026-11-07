#include "servo.h"
#include "stdint.h"
#include "tim.h"

Servo_State servo_state = SERVO_FORWARD;
uint32_t servo_turn_num = 0;
uint32_t value = 0;

void servo_init(void) {
    servo_state = SERVO_FORWARD;
    servo_turn_num = 0;
    value = INIT_VALUE;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value);
}

void servo_turn(void) {
    if (servo_state == SERVO_FORWARD) {
        value += INTERVAL_VALUE;
        servo_turn_num++;
    } else if (servo_state == SERVO_BACKWARD) {
        value -= INTERVAL_VALUE;
        servo_turn_num++;
    }
    if (servo_turn_num + 1 > SERVO_TURN_NUM && servo_state == SERVO_FORWARD) {
        servo_state = SERVO_BACKWARD;
        servo_turn_num = 0;
    } else if (servo_turn_num + 1 > SERVO_TURN_NUM && servo_state == SERVO_BACKWARD) {
        servo_state = SERVO_FORWARD;
        servo_turn_num = 0;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value);
}
