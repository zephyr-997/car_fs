#include "key.h"

Key_t key[4] = {0, 0, 0};

void key_task(void)
{
	if (key[0].flag == 1)
	{
		protection_flag = 1;
		
		key[0].flag = 0;
	}
	if (key[1].flag == 1)
	{
		P52 = !P52;
		
		key[1].flag = 0;
	}
	if (key[2].flag == 1)
	{
		P52 = !P52;
		
		key[2].flag = 0;
	}
	if (key[3].flag == 1)
	{
		P52 = !P52;
		
		key[3].flag = 0;
	}
}