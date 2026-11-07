#include "motor.h"



void Uart_Motor(volatile uint32_t size,char * receiveData,float *motor_target) {
    uint32_t sum=0;
    uint32_t num=0;

    if (motor_flag==1) {
        motor_flag=0;


        for (int i =6;i<size;i++) {
            num = receiveData[i];
            if ((num=='\r') ||(num<'0'||num>'9')){
                break;
            }
            sum = (num-'0')+sum*10;
        }
        if (receiveData[5]=='+') {
            *motor_target=(float)(sum);
        }else if (receiveData[5]=='-') {
            *motor_target=(-1)*(float)(sum);
        }
    }
}
