#ifndef __KEY_H
#define __KEY_H


#include "headfile.h"


typedef struct
{
	uint8 step;
	uint8 state;
	uint8 flag;
} Key_t;


extern Key_t key[4];

void key_task(void);

#endif
