#ifndef __ENCODER_H__
#define __ENCODER_H__


#include "headfile.h"
#include "pid.h"

#define  LEFT_DIR    P35
#define  RIGHT_DIR   P53


typedef struct
{
	int count;
	int encoderlast;
} EncoderDebo_t;


extern EncoderDebo_t EncoderDeboL, EncoderDeboR;


extern int g_encoleft_init, g_encoright_init;
extern int g_EncoderLeft, g_EncoderRight;
extern int g_intencoder;


void encoder_init(void);
int get_left_encoder(void);
int get_right_encoder(void);
int encoder_debounce(EncoderDebo_t* instance, int encoder);


#endif