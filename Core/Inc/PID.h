#ifndef PID_H
#define PID_H
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float integral;
    float last_error;
    float output;
}PID_Typedef;

typedef enum{forwardstate=0,backwardstate,none}Speed_PID_State;

extern PID_Typedef speed_pid;

void PID_Init(PID_Typedef *pid,float kp,float ki,float kd);
float PID_Calculation(PID_Typedef *pid,float target,float actual);
void PID_Reset(PID_Typedef *pid);

#endif //PID_H
