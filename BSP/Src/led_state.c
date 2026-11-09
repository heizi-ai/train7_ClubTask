/**
 * @file    led_state.c
 * @brief   BSP 层 —— C 板 LED 状态指示
 *
 * 逻辑（对应任务 7.4）：
 *   电机在线   → 绿灯常亮，红灯灭
 *   电机离线   → 绿灯灭，红灯做 2 秒周期的呼吸（暗→亮→暗）
 *
 * 硬件说明：
 *   绿灯 PH11 普通 GPIO 输出（高电平点亮）
 *   红灯 PH12 复用为 TIM5_CH3，用 PWM 占空比调亮度
 */

#include "led_state.h"
#include "tim.h"
#include "freertos_os2.h"
#include "freertos_mpool.h"
#include "cmsis_os2.h"
#include "bsp_can.h"

/**
 * @brief  LED 状态机（由 LED 任务每 10ms 调一次）
 * @note   last_time 是 CAN 中断更新的"最近收到反馈的时间戳"，
 *         超过 MOTOR_OFFLINE_TIME(100ms) 没更新就判定电机离线。
 */
void led_state(void) {
    if (HAL_GetTick() - last_time >= MOTOR_OFFLINE_TIME) {
        /* ---------- 离线：绿灯灭 + 红灯呼吸 ---------- */
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

        /* 两段各 1000 步、每步 1ms → 单程 1 秒，往返 2 秒，正好一呼一吸 */
        for (int i = 0; i < 1000; i++) {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, i);   /* 占空比逐渐增大 */
            osDelay(1);                                        /* 必须用 osDelay，不能用 HAL_Delay 占死 CPU */
        }
        for (int i = 0; i < 1000; i++) {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1000-i);
            osDelay(1);
        }
    } else {
        /* ---------- 在线：绿灯常亮 + 红灯灭 ---------- */
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);       /* 占空比 0 = 红灯灭 */
    }
}
