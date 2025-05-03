#include "pid.h"

PID_t LeftPID = { 0.0f , //kp
				  0.0f , //ki
				  0.0f , //kd
				  0.0f , //kf
	              0.0f , //上次误差
				  0.0f , //上上次误差
				  0.0f , //积分误差
				  0.0f , //上次目标值
				  0.0f , //pid输出
				  0.0f , //积分限幅
				  0.0f   //输出限幅
				};//左轮速度环PID

				  
PID_t RightPID = { 0.0f , //kp
				   0.0f , //ki
				   0.0f , //kd
				   0.0f , //kf
	               0.0f , //上次误差
				   0.0f , //上上次误差
				   0.0f , //积分误差
				   0.0f , //上次目标值
	               0.0f , //pid输出
				   0.0f , //积分限幅
				   0.0f   //输出限幅
				 };//右轮速度环PID


PID_t TurnPID = { 0.0f ,   //kp
				  0.0f ,   //ki
				  0.0f ,   //kd
				  0.0f ,   //kf
	              0.0f ,   //上次误差
				  0.0f ,   //上上次误差
				  0.0f ,   //积分误差
				  0.0f ,   //上次目标值
	              0.0f ,   //pid输出
				  0.0f ,   //积分限幅
				  900.0f   //输出限幅
				};//转向环PID

				
float myfabs(float num)
{
	return (num > 0) ? num : -num;
}

int myabs(int num)
{
	return (num > 0) ? num : -num;
}
				

//位置式PID（带前馈）
float pid_poisitional_feedforward(PID_t* pid, float real, float target)
{
	float error = target - real;
	pid->interror += error;
	
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
	pid->output = pid->kp * error + pid->ki * pid->interror + pid->kd * (error - pid->lasterror) + pid->kf * (target - pid->lasttarget);
	
	pid->lasterror = error;
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
	float error = target - real;

	pid->output += pid->kp * (error - pid->lasterror) + pid->ki * error + pid->kd * (error - 2 * pid->lasterror + pid->preverror) + pid->kf * (target - pid->lasttarget);
	
	pid->preverror = pid->lasterror;
	pid->lasterror = error;
	pid->lasttarget = target;
	
	return pid->output;
}


float pid_poisitional_normal(PID_t* pid, float position)
{
	pid->output = pid->kp * position + pid->kd * (position - pid->lasterror);
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


//魔改位置式pid（加二次项）
float pid_poisitional_quadratic(PID_t* pid, float position, float GyroZ)
{
	float error = position;
	
	pid->output = (pid->kp * error) + (pid->kp * pid->kp * error * myfabs(error)) + (pid->kd * (error - pid->lasterror) + (pid->kd * pid->kd * GyroZ));
	pid->lasterror = error;
	
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