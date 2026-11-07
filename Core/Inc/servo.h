#ifndef SERVO_H
#define SERVO_H

#define SERVO_ANGLE 180
#define SERVO_TURN_NUM SERVO_ANGLE/20

typedef enum{servo_forward,servo_backward}Servo_State;
extern Servo_State servo_state;

void servo_init(void);
void servo_turn(void);



#endif //SERVO_H
