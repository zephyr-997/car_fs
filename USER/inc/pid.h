#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	float kf;     //前馈
	
	float lasterror;
	float preverror;
	float interror;
	
	float lasttarget;
	
	float output;
	float i_limit;//积分限幅
	float o_limit;//输出限幅
} PID_t;


extern PID_t LeftPID;
extern PID_t RightPID;
extern PID_t TurnPID;

float pid_poisitional_feedforward(PID_t* pid, float real, float target);
float pid_increment_feedforward(PID_t* pid, float real, float target);
float pid_poisitional_quadratic(PID_t* pid, float position, float GyroZ);

#endif