/**
 * @file    motor.c
 * @brief   BSP 层 —— 解析上位机串口指令 "motor±数字"，写入目标转速
 *
 * 协议：motor+100\r\n  → 正转 100 rpm
 *       motor-100\r\n  → 反转 100 rpm
 * 说明：接收由 DMA + 空闲中断完成（见 Core/Src/freertos.c 的回调），
 *       本函数只负责"解析并赋值"，不碰串口外设。
 */

#include "motor.h"

/**
 * @brief  解析一帧串口指令，更新目标转速
 * @param  size        本帧有效字节数（由串口空闲中断给出）
 * @param  receiveData 接收缓冲区
 * @param  motor_target 目标转速输出（写入 -N 或 +N）
 * @note   只有在 motor_flag == 1（中断已确认帧头合法）时才解析，解析完立刻清零，
 *         保证一帧指令只被处理一次。
 */
void Uart_Motor_SetTarget(volatile uint32_t size, char *receiveData, float *motor_target) {
    uint32_t sum = 0;
    char num = 0;

    if (motor_flag == 1) {
        motor_flag = 0;                       /* 清标志，避免同一帧被反复解析 */

        /* 从第 6 个字符开始（"motor" 占 0~4，符号占 5），逐位拼数字 */
        for (int i = 6; i < size; i++) {
            num = receiveData[i];
            if ((num == '\r') || (num < '0' || num > '9')) {
                break;                        /* 遇到回车/换行/非数字就结束，兼容带不带 \r\n */
            }
            sum = (num - '0') + sum * 10;      /* 逐位左移累加："123" → 1→12→123 */
        }

        if (receiveData[5] == '+') {
            *motor_target = (float) (sum);
        } else if (receiveData[5] == '-') {
            *motor_target = (-1) * (float) (sum);   /* 先转 float 再取负，避免无符号回绕 */
        }
    }
}
