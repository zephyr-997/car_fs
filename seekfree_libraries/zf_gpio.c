/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		gpio
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/


#include "zf_gpio.h"


#define PxPU_BASE_ADDR  0x7EFE10 

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO��������������������
//  @param      pin         ѡ�����ţ�P0_0-P7_7��
//  @param      pull        �������������� NOPULL:������ PULLUP:����
//  @return     void
//  Sample usage:           gpio_pull_set(P0_0,NOPULL);       // ����P0.0����û������������
//-------------------------------------------------------------------------------------------------------------------
void gpio_pull_set(PIN_enum pin, PULL_enum pull)
{
	if(PULLUP == pull)
	{
		(*(unsigned char volatile far *)(PxPU_BASE_ADDR + (pin >> 4))) |= (1<<(pin&0x0F));
	}
	else if(NOPULL == pull)
	{
		(*(unsigned char volatile far *)(PxPU_BASE_ADDR + (pin >> 4))) &= ~(1<<(pin&0x0F));
	}
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO��������ģʽ
//  @param      pin         ѡ�����ţ�P0_0-P5_4��
//  @param      mode        ����ģʽ GPIO:׼˫���, GPO_PP:�������, GPI_IMPEDANCE:��������, GPI_OD:��©���
//  @return     void
//  Sample usage:           gpio_mode(P0_0,GPIO);       // ����P0.0����Ϊ˫��IO
//-------------------------------------------------------------------------------------------------------------------
void gpio_mode(PIN_enum pin, GPIOMODE_enum mode)
{
	if(GPIO == mode)
	{
		if(0x00 == (pin&0xF0))	//P0
		{
			P0M1 &= ~(1<<(pin&0xF));
			P0M0 &= ~(1<<(pin&0xF));
		}
		if(0x10 == (pin&0xF0))	//P1
		{
			P1M1 &= ~(1<<(pin&0xF));
			P1M0 &= ~(1<<(pin&0xF));
		}
		if(0x20 == (pin&0xF0))	//P2
		{
			P2M1 &= ~(1<<(pin&0xF));
			P2M0 &= ~(1<<(pin&0xF));
		}
		
		if(0x30 == (pin&0xF0))	//P3
		{
			P3M1 &= ~(1<<(pin&0xF));
			P3M0 &= ~(1<<(pin&0xF));
		}
		if(0x40 == (pin&0xF0))	//P4
		{
			P4M1 &= ~(1<<(pin&0xF));
			P4M0 &= ~(1<<(pin&0xF));
		}
		if(0x50 == (pin&0xF0))	//P5
		{
			P5M1 &= ~(1<<(pin&0xF));
			P5M0 &= ~(1<<(pin&0xF));
		}
		if(0x60 == (pin&0xF0))	//P5
		{
			P6M1 &= ~(1<<(pin&0xF));
			P6M0 &= ~(1<<(pin&0xF));
		}
		if(0x70 == (pin&0xF0))	//P5
		{
			P7M1 &= ~(1<<(pin&0xF));
			P7M0 &= ~(1<<(pin&0xF));
		}
	}
	else if(GPO_PP == mode)
	{
		if(0x00 == (pin&0xF0))	//P0
		{
			P0M1 &= ~(1<<(pin&0xF));
			P0M0 |=  (1<<(pin&0xF));
		}
		if(0x10 == (pin&0xF0))	//P1
		{
			P1M1 &= ~(1<<(pin&0xF));
			P1M0 |=  (1<<(pin&0xF));
		}
		if(0x20 == (pin&0xF0))	//P2
		{
			P2M1 &= ~(1<<(pin&0xF));
			P2M0 |=  (1<<(pin&0xF));
		}
		
		if(0x30 == (pin&0xF0))	//P3
		{
			P3M1 &= ~(1<<(pin&0xF));
			P3M0 |=  (1<<(pin&0xF));
		}
		if(0x40 == (pin&0xF0))	//P4
		{
			P4M1 &= ~(1<<(pin&0xF));
			P4M0 |=  (1<<(pin&0xF));
		}
		if(0x50 == (pin&0xF0))	//P5
		{
			P5M1 &= ~(1<<(pin&0xF));
			P5M0 |=  (1<<(pin&0xF));
		}
		if(0x60 == (pin&0xF0))	//P4
		{
			P6M1 &= ~(1<<(pin&0xF));
			P6M0 |=  (1<<(pin&0xF));
		}
		if(0x70 == (pin&0xF0))	//P5
		{
			P7M1 &= ~(1<<(pin&0xF));
			P7M0 |=  (1<<(pin&0xF));
		}
		
	}
	else if(GPI_IMPEDANCE == mode)
	{
		if(0x00 == (pin&0xF0))	//P0
		{
			P0M1 |=  (1<<(pin&0xF));
			P0M0 &= ~(1<<(pin&0xF));
		}
		if(0x10 == (pin&0xF0))	//P1
		{
			P1M1 |=  (1<<(pin&0xF));
			P1M0 &= ~(1<<(pin&0xF));
		}
		if(0x20 == (pin&0xF0))	//P2
		{
			P2M1 |=  (1<<(pin&0xF));
			P2M0 &= ~(1<<(pin&0xF));
		}
		
		if(0x30 == (pin&0xF0))	//P3
		{
			P3M1 |=  (1<<(pin&0xF));
			P3M0 &= ~(1<<(pin&0xF));
		}
		if(0x40 == (pin&0xF0))	//P4
		{
			P4M1 |=  (1<<(pin&0xF));
			P4M0 &= ~(1<<(pin&0xF));
		}
		if(0x50 == (pin&0xF0))	//P5
		{
			P5M1 |=  (1<<(pin&0xF));
			P5M0 &= ~(1<<(pin&0xF));
		}
		if(0x60 == (pin&0xF0))	//P5
		{
			P6M1 |=  (1<<(pin&0xF));
			P6M0 &= ~(1<<(pin&0xF));
		}
		if(0x70 == (pin&0xF0))	//P5
		{
			P7M1 |=  (1<<(pin&0xF));
			P7M0 &= ~(1<<(pin&0xF));
		}
	}
	else if(GPI_OD == mode)
	{
		if(0x00 == (pin&0xF0))	//P0
		{
			P0M1 |= (1<<(pin&0xF));
			P0M0 |= (1<<(pin&0xF));
		}
		if(0x10 == (pin&0xF0))	//P1
		{
			P1M1 |= (1<<(pin&0xF));
			P1M0 |= (1<<(pin&0xF));
		}
		if(0x20 == (pin&0xF0))	//P2
		{
			P2M1 |= (1<<(pin&0xF));
			P2M0 |= (1<<(pin&0xF));
		}
		
		if(0x30 == (pin&0xF0))	//P3
		{
			P3M1 |= (1<<(pin&0xF));
			P3M0 |= (1<<(pin&0xF));
		}
		if(0x40 == (pin&0xF0))	//P4
		{
			P4M1 |= (1<<(pin&0xF));
			P4M0 |= (1<<(pin&0xF));
		}
		if(0x50 == (pin&0xF0))	//P5
		{
			P5M1 |= (1<<(pin&0xF));
			P5M0 |= (1<<(pin&0xF));
		}
		if(0x60 == (pin&0xF0))	//P5
		{
			P6M1 |= (1<<(pin&0xF));
			P6M0 |= (1<<(pin&0xF));
		}
		if(0x70 == (pin&0xF0))	//P5
		{
			P7M1 |= (1<<(pin&0xF));
			P7M0 |= (1<<(pin&0xF));
		}
	}
}

