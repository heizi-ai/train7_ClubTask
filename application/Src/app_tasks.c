/**
 * @file    app_tasks.c
 * @brief   应用层 —— 任务主体实现
 *
 * 设计意图：
 *   Core/Src/freertos.c 是 CubeMX 生成的，里面只保留"任务壳子"，
 *   真正的循环逻辑写到这里，这样即使 CubeMX 重新生成代码也不会丢业务逻辑。
 */

#include "app_tasks.h"
#include "cmsis_os2.h"
#include "led_state.h"

/**
 * @brief  LED 任务主体：每 10ms 刷新一次 LED 状态
 * @note   led_state() 内部在"电机离线"分支里会自己做 2 秒的呼吸，
 *         所以这个任务看起来周期很短，实际离线时会一直停在里面。
 */
void APP_LED_TASK(void *argument) {
    for (;;) {
        led_state();
        osDelay(10);
    }
}
