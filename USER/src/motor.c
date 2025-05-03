#include "motor.h"


void motor_init(void)
{
	//左轮初始化
	pwm_init(PWMA_CH1P_P60, MOTOR_PWM_FREQ, 0);
	pwm_init(PWMA_CH2P_P62, MOTOR_PWM_FREQ, 0);
	
	//右轮初始化
	pwm_init(PWMA_CH3P_P64, MOTOR_PWM_FREQ, 0);
	pwm_init(PWMA_CH4P_P66, MOTOR_PWM_FREQ, 0);
}


void set_motor_pwm(int left_duty, int right_duty)
{
	// 左轮PWM限幅并输出
	if(left_duty >= 0)
	{
		if (left_duty > MOTOR_PWM_LIMIT)
		{
			left_duty = MOTOR_PWM_LIMIT;
		}
	
		pwm_duty(PWMA_CH1P_P60, left_duty);
		pwm_duty(PWMA_CH2P_P62, 0);
	}
	else
	{
		if (left_duty < -MOTOR_PWM_LIMIT)
		{
			left_duty = -MOTOR_PWM_LIMIT;
		}
	
		pwm_duty(PWMA_CH1P_P60, 0);
		pwm_duty(PWMA_CH2P_P62, -left_duty);
	}
	
	// 右轮PWM限幅并输出
	if (right_duty >= 0)
	{
		if (right_duty > MOTOR_PWM_LIMIT)
		{
			right_duty = MOTOR_PWM_LIMIT;
		}
		
		pwm_duty(PWMA_CH3P_P64, right_duty);
		pwm_duty(PWMA_CH4P_P66, 0);
	}
	else	
	{
		if (right_duty < -MOTOR_PWM_LIMIT)
		{
			right_duty = -MOTOR_PWM_LIMIT;
		}
		
		pwm_duty(PWMA_CH3P_P64, 0);
		pwm_duty(PWMA_CH4P_P66, -right_duty);
	}
}
