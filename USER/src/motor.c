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


void set_pwm_left(int32_t duty)
{
	//PWM限幅并输出
	if(duty >= 0)
	{
		if (duty > MOTOR_PWM_LIMIT)
		{
			duty = MOTOR_PWM_LIMIT;
		}
	
		pwm_duty(PWMA_CH1P_P60, duty);
	}
	else
	{
		if (duty < -MOTOR_PWM_LIMIT)
		{
			duty = -MOTOR_PWM_LIMIT;
		}
	
		pwm_duty(PWMA_CH2P_P62, -duty);
	}
}


void set_pwm_right(int32_t duty)
{
	if (duty >= 0)
	{
		if (duty > MOTOR_PWM_LIMIT)
		{
			duty = MOTOR_PWM_LIMIT;
		}
		
		pwm_duty(PWMA_CH3P_P64, duty);
	}
	else	
	{
		if (duty < -MOTOR_PWM_LIMIT)
		{
			duty = -MOTOR_PWM_LIMIT;
		}
		
		pwm_duty(PWMA_CH4P_P66, -duty);
	}
}
