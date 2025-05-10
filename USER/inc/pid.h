#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	float kf;     //前馈
	
	float error;
	float lasterror;
	float preverror;
	float interror;
	float lasttarget;
	
	float p_out;
	float i_out;
	float d_out;
	float output;
	
	float i_limit;//积分限幅
	float o_limit;//输出限幅
} PID_t;


extern PID_t LeftPID;
extern PID_t RightPID;
extern PID_t TurnPID;

int myabs(int num);
float myfabs(float num);

void pid_init(PID_t* pid, float kp, float ki, float kd, float kf, float i_limit, float o_limit);
float pid_poisitional_feedforward(PID_t* pid, float real, float target);
float pid_increment_feedforward(PID_t* pid, float real, float target);
float pid_poisitional_normal(PID_t* pid, float position);
float pid_poisitional_quadratic(PID_t* pid, float position, float GyroZ);

#endif