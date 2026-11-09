#ifndef PID_H
#define PID_H

#define PID_OUTPUT_MAX 20000
#define PID_OUTPUT_MIN (-20000)

typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float integral;
    float last_error;
    float output;
} PID_Typedef;

typedef enum { SPEED_PID_FORWARD = 0, SPEED_PID_BACKWARD, SPEED_PID_NONE } Speed_PID_State;

extern PID_Typedef speed_pid;

void PID_Init(PID_Typedef *pid, float kp, float ki, float kd);

float PID_Calculation(PID_Typedef *pid, float target, float actual);

void PID_Reset(PID_Typedef *pid);

#endif //PID_H
