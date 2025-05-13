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
int32_t g_DutyLeft = 0, g_DutyRight = 0;         // 最后真正要给电机的PWM值
float Gyro_Z = 0, filtered_GyroZ = 0;            // 陀螺仪角速度的原始值和卡尔曼滤波之后的值
float turn_pid = 0;
int g_SpeedPoint = 30;
int g_LeftPoint = 0;                             // 左轮目标速度                  
int g_RightPoint = 0;                            // 右轮目标速度             
int count = 0, flag = 0;
int turn_count = 0;

float k = 0;
int turnflag = 0;
uint8_t startKeyFlag = 0, uartSendFlag = 1;

// 蜂鸣器控制相关变量
uint8_t beep_flag = 0;          // 蜂鸣器开启标志，1表示开启
uint16_t beep_count = 0;        // 蜂鸣器计时计数器
uint8_t track_ten_cnt = 0;    //出入环重复判定计时器


extern uint8 track_ten_flag;
extern uint8 ten_change_flag;

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
	
	/* 普通定时功能，备用 */
	count++;
	if (count >= 50)
	{
		flag = 1;
		count = 0;
	}

    /* 检测赛道类型变化并控制蜂鸣器 */
    if (track_type != track_type_last)
    {
        // 赛道类型发生变化，启动蜂鸣器
        beep_flag = 1;
        beep_count = 0;  // 重置计数器
        P26 = 0;  // 打开蜂鸣器
        
        // 更新上一次赛道类型
        track_type_last = track_type;
    }
    
    /* 蜂鸣器计时控制 */
    if (beep_flag)
    {
        beep_count++;
        // 10ms * 20 = 200ms
        if (beep_count >= 10)
        {
            beep_count = 0;
            beep_flag = 0;
            P26 = 1;  // 关闭蜂鸣器
        }
    }	
	
	/* 出入十字圆环计时判定 */
	if (ten_change_flag == 1)
	{
		track_ten_cnt++;
		if (track_ten_cnt >= 150)
		{
			track_ten_flag = 1;
			track_ten_cnt = 0;
			ten_change_flag = 0;
		}
	}
}

//定时器2中断
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
	//读取并清除编码器的值
	g_encoleft_init = get_left_encoder();
	g_encoright_init = get_right_encoder();
	
	imu963ra_get_gyro();
	Gyro_Z = imu963ra_gyro_transition(imu963ra_gyro_z);
	

	if (startKeyFlag == 1)
	{
		/* 对编码器的值进行滤波 */
		g_EncoderLeft = LowPass_Filter(&leftSpeedFilt, g_encoleft_init);
		g_EncoderRight = LowPass_Filter(&rightSpeedFilt, g_encoright_init);
		
		/* 对编码器的值进行异常消除 */
		g_EncoderLeft = encoder_debounce(&EncoderDeboL, g_EncoderLeft);
		g_EncoderRight = encoder_debounce(&EncoderDeboR, g_EncoderRight);
		
		
		if (track_type == 0 || track_type == 1 || track_type == 2 || (track_type == 3 && track_route_status == 2))//普通直线、直角、十字圆环内部或者圆环内部
		{
			/* 5ms算一次内环，15ms算一次外环 */
			turn_count++;
			if (turn_count >= 3)
			{
				filtered_GyroZ = Kalman_Update(&imu693_kf, Gyro_Z);//对Gyro_Z进行卡尔曼滤波
				
				turn_pid = pid_poisitional_normal(&TurnPID, position);
//				turn_pid = pid_poisitional_quadratic(&TurnPID, position, filtered_GyroZ);
				
				Kalman_Predict(&imu693_kf, turn_pid);//更新卡尔曼滤波器的值
				
				turn_count = 0;
			}
			
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
			
//			if (track_type == 1)//直角
//			{
//				if (track_type_zj == 1)//左转直角，积分积右轮
//				{
//					g_intencoderR += g_EncoderRight;
//					
//					if (g_intencoderR >= 2900)
//					{
//						g_intencoderR = 0;
//						track_type = 0; 
//						track_type_zj = 0;
//					}
//				}
//				else if (track_type_zj == 2)//右转直角，积分积左轮
//				{
//					g_intencoderL += g_EncoderLeft;
//					
//					if (g_intencoderL >= 2500)
//					{
//						g_intencoderL = 0;
//						track_type = 0; 
//						track_type_zj = 0;
//					}
//				}
//			}
		}
		else if (track_type == 3 && track_route_status == 1)//圆环入环
		{
			g_intencoderALL += ((g_EncoderLeft + g_EncoderRight) / 2);
			
			if(g_intencoderALL <= 4000)//第一阶段先直行
			{
				g_LeftPoint = g_SpeedPoint;
				g_RightPoint = g_SpeedPoint;
			}
			else//进入第二阶段打死进环
			{
				if (track_route == 1)//右环
				{
					g_LeftPoint = g_SpeedPoint * 1.4;
					g_RightPoint = g_SpeedPoint * 0.7;
				}
				else if (track_route == 2)//左环
				{
					g_LeftPoint = g_SpeedPoint * 0.8;
					g_RightPoint = g_SpeedPoint * 1.3;
				}
							
				if (g_intencoderALL >= 8500)//入环完毕
				{
					track_route_status = 2;
					g_intencoderALL = 0;
				}
			}
		}
		else if (track_type == 3 && track_route_status == 3)//圆环出环
		{
			g_intencoderALL += (g_EncoderLeft + g_EncoderRight) / 2;
			
			if (g_intencoderALL <= 4400)//第一阶段打死出环
			{
				if (track_route == 1)//右环
				{
					g_LeftPoint = g_SpeedPoint * 1.30;
					g_RightPoint = g_SpeedPoint * 0.80;
				}
				else if (track_route == 2)//左环
				{
					g_LeftPoint = g_SpeedPoint * 0.8;
					g_RightPoint = g_SpeedPoint * 1.25;
				}
			}
			else//第二阶段直走
			{
				g_LeftPoint = g_SpeedPoint;
				g_RightPoint = g_SpeedPoint;
				
				if (g_intencoderALL >= 5600)//出环完毕
				{
					track_type = 0;
					track_route = 0;
					track_route_status = 0;
					
					g_intencoderALL = 0;
				}
			}
		}
		
		//计算速度环pid
		left_pid = pid_increment_feedforward(&LeftPID, g_EncoderLeft, g_LeftPoint);
		right_pid = pid_increment_feedforward(&RightPID, g_EncoderRight, g_RightPoint);
		
		//转int
		g_DutyLeft = (int32_t)left_pid;
		g_DutyRight = (int32_t)right_pid;
	
		if (protection_flag == 1)
		{
			LeftPID.output = LeftPID.lasterror = LeftPID.preverror = 0;
			RightPID.output = RightPID.lasterror = RightPID.preverror = 0;
			uartSendFlag = 0;
			
			set_motor_pwm(0, 0);
		}
		else
		{
			set_motor_pwm(g_DutyLeft, g_DutyRight);
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