#include "key.h"

Key_t key[4] = {0, 0, 0};

void key_task(void)
{
	if (key[0].flag == 1)
	{
		if (startKeyFlag == 1)
		{
			set_motor_pwm(0, 0);
			
			TurnPID.lasterror = TurnPID.interror = 0;

			LeftPID.output = LeftPID.lasterror = LeftPID.preverror = 0;
			
			RightPID.output = RightPID.lasterror = RightPID.preverror = 0;
			
			uartSendFlag = startKeyFlag = 0;
		}
		else
		{
			delay_ms(2000);
			uartSendFlag = startKeyFlag = 1;
		}
		
		key[0].flag = 0;
	}
	if (key[1].flag == 1)
	{

		
		key[1].flag = 0;
	}
	if (key[2].flag == 1)
	{

		
		key[2].flag = 0;
	}
	if (key[3].flag == 1)
	{

		
		key[3].flag = 0;
	}
}