// cursor-ignore-file
///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,逐飞科技
// * All rights reserved.
// * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
// *
// * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
// * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
// *
// * @file       		isr
// * @company	   		成都逐飞科技有限公司
// * @author     		逐飞科技(QQ790875685)
// * @version    		查看doc内version文件 版本说明
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32G12K128
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"
#include "encoder.h"
#include "filter.h"
#include "motor.h"
#include "pid.h"
#include "isr.h"
#include "key.h"
#include "electromagnetic_tracking.h"


//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;

// 函数前置声明
void uart4_interrupt_callback(void);

// 全局变量定义
float left_pid = 0, right_pid = 0;               // 速度环pid的增量，还需转化再赋给电机
int g_DutyLeft = 0, g_DutyRight = 0;         // 最后真正要给电机的PWM值
float Gyro_Z = 0, filtered_GyroZ = 0;            // 陀螺仪角速度的原始值和卡尔曼滤波之后的值
float turn_pid = 0;
int g_SpeedPoint = 30;
int g_LeftPoint = 0;                             // 左轮目标速度                  
int g_RightPoint = 0;                            // 右轮目标速度             
int count = 0, flag = 0;
int turn_count = 0;

int k = 0;
int turnflag = 0;
uint8_t startKeyFlag = 0, uartSendFlag = 1;

//UART1中断
void UART1_Isr() interrupt 4
{
//    uint8 res;
//	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
//        res = SBUF;
//        //程序自动下载
//        if(res == 0x7F)
//        {
//            if(dwon_count++ > 20)
//                IAP_CONTR = 0x60;
//        }
//        else
//        {
//            dwon_count = 0;
//        }
    }
}



//UART2中断
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//接收数据寄存器为：S2BUF


	}
}


//UART3中断
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//接收数据寄存器为：S3BUF

	}
}



//UART4中断
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;
		//接收数据寄存器为：S4BUF;

		g_RxDat = S4BUF;
		g_RxData[g_RxPointer++] = g_RxDat;
	}
}



//外部中断0
void INT0_Isr() interrupt 0
{

}



//外部中断1
void INT1_Isr() interrupt 2
{

}



//外部中断2
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //清除中断标志
	
}



//外部中断3
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //清除中断标志
	
}



//外部中断4
void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //清除中断标志
	
}



//定时器0中断
void TM0_Isr() interrupt 1
{
	
}



//定时器1中断
void TM1_Isr() interrupt 3
{
	int i = 0;
	
	key[0].state = P70;
	key[1].state = P71;
	key[2].state = P72;
	key[3].state = P73;
	
	for (i = 0; i < 4; i++)
	{
		switch (key[i].step)
		{
			case 0:
			{
				if (key[i].state == 0)
				{
					key[i].step = 1;
				}
			}
			break;
			
			case 1:
			{
				if (key[i].state == 0)
				{
					key[i].step = 2;
					key[i].flag = 1;
				}
				else
				{
					key[i].step = 0;
				}
			}
			break;
			
			case 2:
			{
				if (key[i].state == 1)
				{
					key[i].step = 0;
				}
			}
			break;
		}
	}
}



//定时器2中断
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
	/* 普通定时功能，备用 */
	count++;
	if (count >= 100)
	{
		flag = 1;
		count = 0;
	}
	
	
	//读取并清除编码器的值
	g_EncoderLeft = get_left_encoder();
	g_EncoderRight = get_right_encoder();
	
	/*
		读取角速度并转化为实际物理数据
		当突然左转，Gyro_Z为正值；突然右转，Gyro_Z为负值
	*/
//	imu963ra_get_gyro();
//	Gyro_Z = imu963ra_gyro_transition(imu963ra_gyro_z);
	
	//对Gyro_Z进行卡尔曼滤波
//	filtered_GyroZ = Kalman_Update(&imu693_kf, Gyro_Z);

	if (startKeyFlag == 1)
	{
		/* 5ms算一次内环，15ms算一次外环 */
		turn_count++;
		if (turn_count >= 3)
		{
			turn_pid = pid_poisitional_normal(&TurnPID, position);
			turn_count = 0;
		}
	
	
	//更新卡尔曼滤波的值
//	Kalman_Predict(&imu693_kf, turn_pid);

		if(turn_pid >= 0) // 左转
		{
			k = turn_pid * 0.01; // 缩放至 0.0 ~ 1.0
			g_LeftPoint = g_SpeedPoint * (1 - k);
			g_RightPoint = g_SpeedPoint * (1 + k * 0.5); // 加少减多
		}
		else // 右转
		{
			k = -turn_pid * 0.01; // 取相反数并缩放至 0.0 ~ 1.0
			g_LeftPoint = g_SpeedPoint * (1 + k * 0.5); // 加少减多
			g_RightPoint = g_SpeedPoint * (1 - k);
		}

		//计算速度环pid
		left_pid = pid_increment_feedforward(&LeftPID, g_EncoderLeft, g_LeftPoint);
		right_pid = pid_increment_feedforward(&RightPID, g_EncoderRight, g_RightPoint);
		
		//转int
		g_DutyLeft = (int)left_pid;
		g_DutyRight = (int)right_pid;
		
		
		if (protection_flag == 0)
		{
			set_motor_pwm(g_DutyLeft, g_DutyRight);
		}
		else
		{
			set_motor_pwm(0, 0);
		}
	}
	
	
}



//定时器3中断
void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //清除中断标志
	
}



//定时器4中断
void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //清除中断标志


}


//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;