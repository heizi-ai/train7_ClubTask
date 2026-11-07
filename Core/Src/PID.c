#include "PID.h"

PID_Typedef speed_pid;
Speed_PID_State speed_pid_state;
int I_limit_state = 0;

void PID_Init(PID_Typedef *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->error = 0;
}

float PID_Calculation(PID_Typedef *pid, float target, float actual) {
    pid->error = target - actual;
    if (pid->error > 0) {
        speed_pid_state = SPEED_PID_FORWARD;
    } else if (pid->error < 0) {
        speed_pid_state = SPEED_PID_BACKWARD;
    } else {
        speed_pid_state = SPEED_PID_NONE;
    }
    float P = pid->kp * pid->error;

    float I = pid->ki * pid->integral;

    float D = pid->kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;

    pid->output = P + I + D;
    if (pid->output > PID_OUTPUT_MAX) {
        pid->output = PID_OUTPUT_MAX;
        if (speed_pid_state == SPEED_PID_FORWARD) {
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
        I_limit_state = 0;
        speed_pid_state = SPEED_PID_NONE;
    }

    if (I_limit_state == 0) {
        pid->integral += pid->error;
        I = pid->ki * pid->integral;
        pid->output = P + I + D;
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

void PID_Reset(PID_Typedef *pid) {
    pid->integral = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->output = 0;
}
