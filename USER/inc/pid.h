#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float i_limit;//积分限幅
	float o_limit;//输出限幅
} PID_t;


extern PID_t LeftPID;
extern PID_t RightPID;
extern PID_t IMU693PID;

float pid_poisitional(PID_t* pid, float now, float point);
float pid_increment(PID_t* pid, int now, int point);

#endif