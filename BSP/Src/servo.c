/**
 * @file    servo.c
 * @brief   BSP 层 —— 舵机 PWM 控制（TIM1_CH1，50Hz）
 *
 * 角度换算：
 *   PWM 周期 20ms（ARR=20000，计数频率 1MHz）
 *   舵机行程 180°，对应脉宽 500µs ~ 2500µs
 *   每按一次按键转 20°，对应脉宽增量见 servo.h 的 INTERVAL_VALUE
 */

#include "servo.h"
#include "stdint.h"
#include "tim.h"

Servo_State servo_state = SERVO_FORWARD;   /* 当前转动方向 */
uint32_t servo_turn_num = 0;               /* 当前方向已经走了几步，用于到头反向 */
uint32_t value = 0;                        /* 当前脉宽（单位 µs，直接写进比较寄存器） */

/**
 * @brief  舵机初始化：回到起点并输出初始脉宽
 * @note   只在舵机任务启动时调用一次；若每次按键都调会把步数清零，导致永远单方向转
 */
void servo_init(void) {
    servo_state = SERVO_FORWARD;
    servo_turn_num = 0;
    value = INIT_VALUE;                                  /* 500µs，舵机 0° 位置 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value);
}

/**
 * @brief  转一步（20°），走到行程尽头自动反向
 */
void servo_turn(void) {
    if (servo_state == SERVO_FORWARD) {
        value += INTERVAL_VALUE;        /* 正向：脉宽 +222µs ≈ +20° */
        servo_turn_num++;
    } else if (servo_state == SERVO_BACKWARD) {
        value -= INTERVAL_VALUE;        /* 反向：脉宽 -222µs */
        servo_turn_num++;
    }

    /* 走到步数上限就反向；+1 是提前一步换向，避免顶到机械限位 */
    if (servo_turn_num + 1 > SERVO_TURN_NUM && servo_state == SERVO_FORWARD) {
        servo_state = SERVO_BACKWARD;
        servo_turn_num = 0;
    } else if (servo_turn_num + 1 > SERVO_TURN_NUM && servo_state == SERVO_BACKWARD) {
        servo_state = SERVO_FORWARD;
        servo_turn_num = 0;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value);
}
