#include "PID.h"

PID_Typedef speed_pid;
Speed_PID_State  speed_pid_state;
int I_limit_state=0;

void PID_Init(PID_Typedef *pid,float kp,float ki,float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->error = 0;
}

float PID_Calculation(PID_Typedef *pid,float target,float actual) {
    pid->error = target - actual;
    if (pid->error>0) {
        speed_pid_state = forwardstate;
    }else if (pid->error<0) {
        speed_pid_state = backwardstate;
    }else {
        speed_pid_state = none;
    }
    float P=pid->kp*pid->error;

    float I=pid->ki*pid->integral;

    float D=pid->kd*(pid->error-pid->last_error);
    pid->last_error = pid->error;

    pid->output=P+I+D;
    if (pid->output>20000) {
        pid->output=20000;
        if (speed_pid_state==forwardstate) {
            speed_pid_state = none;
            I_limit_state = 1;
        }else {
            I_limit_state = 0;
            speed_pid_state = none;
        }
    }else if (pid->output<-20000) {
        pid->output=-20000;
        if (speed_pid_state==backwardstate) {
            speed_pid_state = none;
            I_limit_state = 1;
        }else {
            I_limit_state = 0;
            speed_pid_state = none;
        }
    }else {
        I_limit_state = 0;
        speed_pid_state = none;
    }

    if (I_limit_state==0) {
        pid->integral += pid->error;
        I=pid->ki*pid->integral;
        pid->output=P+I+D;
        if (pid->output>20000) {
            pid->output=20000;
            if (pid->error>0) {
                pid->integral -= pid->error;
            }
        }else if (pid->output<-20000) {
            pid->output=-20000;
            if (pid->error<0) {
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
