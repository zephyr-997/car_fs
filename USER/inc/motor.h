#ifndef __MOTOR_H___
#define __MOTOR_H___


#include "headfile.h"
#include "pid.h"


#define  MOTOR_PWM_FREQ   17000   //PWM输出频率
#define  MOTOR_PWM_LIMIT  8500    //PWM输出限幅：85%


void motor_init(void);
void set_motor_pwm(int32_t left_duty, int32_t right_duty);



#endif