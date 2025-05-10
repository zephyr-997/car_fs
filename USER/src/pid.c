#include "pid.h"


PID_t LeftPID;//左轮速度环PID	  
PID_t RightPID;//右轮速度环PID
PID_t TurnPID;//转向环PID

float myfabs(float num)
{
	return (num > 0) ? num : -num;
}

int myabs(int num)
{
	return (num > 0) ? num : -num;
}

void pid_init(PID_t* pid, float kp, float ki, float kd, float kf, float i_limit, float o_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->kf = kf;

	pid->i_limit = i_limit;
	pid->o_limit = o_limit;	
	
	pid->error = 0.0f;
	pid->lasterror = 0.0f;
	pid->preverror = 0.0f;
	pid->interror = 0.0f;
	pid->lasttarget = 0.0f;
	
	pid->p_out = 0.0f;
	pid->i_out = 0.0f;
	pid->d_out = 0.0f;
	pid->output = 0.0f;
}
				

//位置式PID（带前馈）
float pid_poisitional_feedforward(PID_t* pid, float real, float target)
{
	pid->error = target - real;
	pid->interror += pid->error;
	
	//积分限幅
	if (pid->interror > pid->i_limit)
	{
		pid->interror = pid->i_limit;
	}
	else if (pid->interror < -pid->i_limit)
	{
		pid->interror = -pid->i_limit;
	}
	
	//线性、积分、微分、前馈共同作用
	pid->output = pid->kp * pid->error + pid->ki * pid->interror + pid->kd * (pid->error - pid->lasterror) + pid->kf * (target - pid->lasttarget);
	
	pid->lasterror = pid->error;
	pid->lasttarget = target;
	
	//输出限幅
	if (pid->output > pid->o_limit)
	{
		pid->output = pid->o_limit;
	}
	else if (pid->output < -pid->o_limit)
	{
		pid->output = -pid->o_limit;
	}
	
	return pid->output; 
}


//增量式PID（带前馈）
float pid_increment_feedforward(PID_t* pid, float real, float target)
{
	pid->error = target - real;
	
	pid->p_out = pid->kp * (pid->error - pid->lasterror);
	pid->i_out = pid->ki * pid->error;
	pid->d_out = pid->kd * (pid->error - 2 * pid->lasterror + pid->preverror);

	pid->output += pid->p_out + pid->i_out + pid->d_out + pid->kf * (target - pid->lasttarget);
	
	pid->preverror = pid->lasterror;
	pid->lasterror = pid->error;
	pid->lasttarget = target;
	
	if (pid->output > pid->o_limit)
	{
		pid->output = pid->o_limit;
	}
	else if (pid->output < -pid->o_limit)
	{
		pid->output = -pid->o_limit;
	}
	
	return pid->output;
}


float pid_poisitional_normal(PID_t* pid, float position)
{
	pid->error = position;
	
	pid->p_out = pid->kp * pid->error;
	pid->d_out = pid->kd * (pid->error - pid->lasterror);

	pid->output = pid->p_out + pid->d_out;

	pid->lasterror = pid->error;
	
	//输出限幅
	if (pid->output > pid->o_limit)
	{
		pid->output = pid->o_limit;
	}
	else if (pid->output < -pid->o_limit)
	{
		pid->output = -pid->o_limit;
	}
	
	return pid->output;
}


//魔改位置式pid（加二次项）
float pid_poisitional_quadratic(PID_t* pid, float position, float GyroZ)
{
	pid->p_out = (pid->kp * position) + (pid->kp * pid->kp * position * myfabs(position));
	pid->d_out = (pid->kd * (position - pid->lasterror) + (pid->kd * pid->kd * GyroZ));
	
	pid->output = pid->p_out + pid->d_out;
	pid->lasterror = position;
	
	//输出限幅
	if (pid->output > pid->o_limit)
	{
		pid->output = pid->o_limit;
	}
	else if (pid->output < -pid->o_limit)
	{
		pid->output = -pid->o_limit;
	}
	
	return pid->output;
}