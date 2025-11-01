#ifndef _MOTOR_H_
#define _MOTOR_H_


#define OUT1 		PAout(4)
#define OUT2 		PAout(5)
#define OUT3 		PAout(6)
#define OUT4 		PAout(7)

void InitMotor(void);
void MotorControl(int8 m1Speed,int8 m2Speed);

void MotorPwmCtrl(void);


#endif
