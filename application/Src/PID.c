/**
 * @file    PID.c
 * @brief   应用层 —— 增量式/位置式 PID 计算（速度环）
 *
 * 说明：
 *   这是标准位置式 PID：output = Kp*e + Ki*∫e + Kd*de
 *   里面额外做了两件事：
 *     1) 判断是否处于饱和状态（输出顶到上限/下限）
 *     2) 抗积分饱和：饱和时冻结积分，或把本次积分回滚掉
 */

#include "PID.h"

/* 速度环 PID 实例 */
PID_Typedef speed_pid;
Speed_PID_State speed_pid_state;
int I_limit_state = 0;      /* 1 = 本次输出已饱和，积分需要冻结 */


/**
 * @brief  PID 参数初始化（必须在使用前调用一次，否则 kp/ki/kd 全是 0）
 */
void PID_Init(PID_Typedef *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->error = 0;
}


/**
 * @brief  执行一次 PID 计算
 * @param  pid     PID 实例
 * @param  target  目标值（目标转速 rpm）
 * @param  actual  实际值（反馈转速 rpm）
 * @retval 限幅后的输出（电压 mV）
 */
float PID_Calculation(PID_Typedef *pid, float target, float actual) {
    pid->error = target - actual;

    /* 记录误差方向：用于后面判断"饱和方向是否与误差一致" */
    if (pid->error > 0) {
        speed_pid_state = SPEED_PID_FORWARD;
    } else if (pid->error < 0) {
        speed_pid_state = SPEED_PID_BACKWARD;
    } else {
        speed_pid_state = SPEED_PID_NONE;
    }

    /* ---------- 三个分量（I 用的是上一轮累积的积分） ---------- */
    float P = pid->kp * pid->error;

    float I = pid->ki * pid->integral;

    float D = pid->kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;          /* 本轮误差留到下一轮算微分 */

    /* ---------- 第一段限幅：判断是否饱和 ---------- */
    pid->output = P + I + D;
    if (pid->output > PID_OUTPUT_MAX) {
        pid->output = PID_OUTPUT_MAX;
        if (speed_pid_state == SPEED_PID_FORWARD) {
            /* 误差为正还把输出顶满 → 积分再涨也没用，冻结积分（抗积分饱和） */
            speed_pid_state = SPEED_PID_NONE;
            I_limit_state = 1;
        } else {
            I_limit_state = 0;
            speed_pid_state = SPEED_PID_NONE;
        }
    } else if (pid->output < PID_OUTPUT_MIN) {
        pid->output = PID_OUTPUT_MIN;
        if (speed_pid_state == SPEED_PID_BACKWARD) {
            speed_pid_state = SPEED_PID_NONE;
            I_limit_state = 1;
        } else {
            I_limit_state = 0;
            speed_pid_state = SPEED_PID_NONE;
        }
    } else {
        /* 没饱和，积分可以正常累加 */
        I_limit_state = 0;
        speed_pid_state = SPEED_PID_NONE;
    }

    /* ---------- 第二段：积分并再次限幅 ---------- */
    if (I_limit_state == 0) {
        pid->integral += pid->error;       /* 10ms 一次，所以积分步长就是"每周期累加一次误差" */
        I = pid->ki * pid->integral;
        pid->output = P + I + D;

        /* 加上积分后可能又饱和了，这时把刚才这次积分回滚，避免继续累积 */
        if (pid->output > PID_OUTPUT_MAX) {
            pid->output = PID_OUTPUT_MAX;
            if (pid->error > 0) {
                pid->integral -= pid->error;
            }
        } else if (pid->output < PID_OUTPUT_MIN) {
            pid->output = PID_OUTPUT_MIN;
            if (pid->error < 0) {
                pid->integral -= pid->error;
            }
        }
    }

    return pid->output;
}


/**
 * @brief  清空 PID 内部状态（切换目标/重新启动时用）
 */
void PID_Reset(PID_Typedef *pid) {
    pid->integral = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->output = 0;
}
