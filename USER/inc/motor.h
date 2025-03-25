#ifndef __MOTOR_H___
#define __MOTOR_H___


#include "headfile.h"
#include "pid.h"


#define  MOTOR_PWM_FREQ   17000   //PWM输出频率
#define  MOTOR_PWM_LIMIT  6500     //PWM输出限幅：65%


void motor_init(void);
void set_pwm_left(int32_t duty);
void set_pwm_right(int32_t duty);


#endif