#include "encoder.h"


int g_EncoderLeft = 0, g_EncoderRight = 0;


void encoder_init(void)
{
	ctimer_count_init(CTIM0_P34);
	ctimer_count_init(CTIM3_P04);
}


int get_left_encoder(void)
{
	int encoder_left;
	
	if(LEFT_DIR == 1)
	{
		encoder_left = ctimer_count_read(CTIM0_P34);
	}
	else
	{
		encoder_left = ctimer_count_read(CTIM0_P34) * -1;
	}
	
	ctimer_count_clean(CTIM0_P34);
	
	return encoder_left;
}


int get_right_encoder(void)
{
	int encoder_right;
	
	if(RIGHT_DIR == 1)
	{
		encoder_right = ctimer_count_read(CTIM3_P04) * -1;
	}
	else
	{
		encoder_right = ctimer_count_read(CTIM3_P04);
	}
	
	ctimer_count_clean(CTIM3_P04);
	
	return encoder_right;
}