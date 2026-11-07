#include "led_state.h"
#include "tim.h"
#include "freertos_os2.h"
#include "freertos_mpool.h"
#include "cmsis_os2.h"
#include "bsp_can.h"

void led_state(void) {
    if (HAL_GetTick() - last_time >= MOTOR_OFFLINE_TIME) {
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        for (int i = 0; i < 1000; i++) {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, i);
            osDelay(1);
        }
        for (int i = 0; i < 1000; i++) {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1000-i);
            osDelay(1);
        }
    } else {
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    }
}
