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

	
}


float left_pid = 0, right_pid = 0;               //速度环pid的增量，还需转化再赋给电机
float pidtopwm_left = 0, pidtopwm_right = 0;     //速度环pid的转化值
int g_DutyLeft = 0, g_DutyRight = 0;             //最后真正要给电机的PWM值

float imu693_pid = 0;                            //陀螺仪pid的值
float Gyro_Z = 0, filtered_GyroZ = 0;            //陀螺仪角速度的原始值和卡尔曼滤波之后的值

int g_LeftPoint = 50;                            //左轮目标速度                  
int g_RightPoint = 50;                           //右轮目标速度       
float g_IMU693Point = 0.0;                       //陀螺仪目标角速度       

int count = 0, flag = 0;


//定时器2中断
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
	count++;
	if (count >= 300)
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
	imu963ra_get_gyro();
	Gyro_Z = imu963ra_gyro_transition(imu963ra_gyro_z);
	
	//对Gyro_Z进行卡尔曼滤波
	filtered_GyroZ = Kalman_Update(&imu693_kf, Gyro_Z);
	
	//计算陀螺仪角速度pid
	imu693_pid = pid_poisitional(&IMU693PID, filtered_GyroZ, g_IMU693Point);
	
	//更新卡尔曼滤波的值
	Kalman_Predict(&imu693_kf, imu693_pid);
	
	//计算速度环pid
	left_pid = pid_increment(&LeftPID, g_EncoderLeft, g_LeftPoint);
	right_pid = pid_increment(&RightPID, g_EncoderRight, g_RightPoint);
	
	/*
		把速度环pid的值转化成PWM的增值
		因为left_pid和right_pid的值很小，大概在零点几左右，所以我就把他放大了一点
		所以也不是严格的转化成pwm的数量级
	*/
	pidtopwm_left = 40.0 * left_pid;
	pidtopwm_right = 40.0 * right_pid;
	
	//并级pid累加
	g_DutyLeft += pidtopwm_left - imu693_pid;
	g_DutyRight += pidtopwm_right + imu693_pid;
	
	// set_motor_pwm(g_DutyLeft, g_DutyRight);

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