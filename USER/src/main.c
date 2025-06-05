#include "headfile.h"

// 选择使用传统ADC方式或DMA ADC方式
#define USE_DMA_ADC 1  // 1表示使用DMA ADC，0表示使用传统ADC
#define DMA_ADC_TEST_MODE  1

extern uint8_t track_ten_cnt;
extern volatile uint8 g_adc_dma_completed_flag;  // 引用DMA ADC数据就绪标志


void main(void)
{
	int state = 5;
	uint16 sum_value = 0;    //
	uint16 value[7] = {0};   //调试用数组
	
	board_init();			

	// ips114_init_simspi();					
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_4);

	uart_putstr(UART_4, "test0...\r\n");
	motor_init();
	encoder_init();
	
	imu963ra_init();
	
//	pid_init(&LeftPID, 0.0f, 0.0f, 0.0f, 0.0f, 5000.0f, 6000.0f);
//	pid_init(&RightPID, 0.0f, 0.0f, 0.0f, 0.0f, 5000.0f, 6000.0f);
//	pid_init(&TurnPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 250.0f);
	
	pid_init(&SpeedPID, 110.0f, 0.6f, 0.0f, 0.0f, 5000.0f, 6000.0f);
	pid_init(&TurnPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6000.0f);
	
	LowPass_init(&leftSpeedFilt, 0.556);   //初始化低通滤波器
	LowPass_init(&rightSpeedFilt, 0.556);
	
	Kalman_Init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);
	
	pit_timer_ms(TIM_1, 10);
	pit_timer_ms(TIM_2, 5);
	
	// ips114_clear_simspi(WHITE);	 //清屏
	delay_ms(100); // 延时等待系统稳定

	// 初始化电磁传感器
#if USE_DMA_ADC

	electromagnetic_dma_init();     // 初始化DMA ADC电磁传感器
	// 启动第一次DMA ADC转换
	start_adc_dma_conversion();

#endif


    // 添加DMA测试功能入口
#ifdef DMA_ADC_TEST_MODE

	// 运行DMA ADC测试
	run_electromagnetic_dma_tests();

#endif
	
    while(1)
	{		
		/* 串口接收 */
		if(g_RxPointer != 0)
		{
			int temp = g_RxPointer;
			delay_ms(4);
			if(temp == g_RxPointer)
			{
				uart4_interrupt_callback();
			}
		}
		
		
		/* 定时操作 */
		if (flag == 1)
		{
//			if (g_SpeedPoint == 20)
//			{
//				g_SpeedPoint = 70;
//			}
//			else if (g_SpeedPoint == 70)
//			{
//				g_SpeedPoint = 20;
//			}
			
			flag = 0;
		}
		
		/* 按键处理 */
		key_task();

		if (uartSendFlag == 1)
		{
#if 0
			sprintf(g_TxData,"%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d,%d\n",
					g_LeftPoint,
					g_EncoderLeft,
					g_RightPoint,
					g_EncoderRight,
					position,
					(int)turn_pid,
					g_DutyLeft,
					g_DutyRight,
					track_type,
					track_route,
					track_route_status);
			uart_putstr(UART_4, g_TxData);
#endif
					
#if 0
			sprintf(g_TxData,"%d,%d,%d,%d,%d,%ld,%ld,%d,%d,%d\n",
					g_SpeedPoint,
					g_EncoderAverage,
					(int)speed_pid,
					position,
					(int)turn_pid,
					g_DutyLeft,
					g_DutyRight,
					(int)SpeedPID.interror,
					(int)SpeedPID.p_out,
					(int)SpeedPID.i_out);
			uart_putstr(UART_4, g_TxData);
#endif
					
//			sprintf(g_TxData,"%d,%d,%d,%d\n",
//					g_LeftPoint,
//					g_EncoderLeft,
//					g_RightPoint,
//					g_EncoderRight);
//			uart_putstr(UART_4, g_TxData);
					
//			sprintf(g_TxData,"%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\n",
//					(float)g_LeftPoint,
//					(float)g_EncoderLeft,
//					(float)g_DutyLeft,
//					LeftPID.error,
//					LeftPID.lasterror,
//					LeftPID.p_out,
//					LeftPID.output
//			);
//			uart_putstr(UART_4, g_TxData);
			
//			sprintf(g_TxData,"%d,%d,%d,%d\n",g_encoleft_init,g_encoright_init,g_EncoderLeft,g_EncoderRight);
//			uart_putstr(UART_4, g_TxData);
			
//			sprintf(g_TxData, "%f,%f\n",Gyro_Z,filtered_GyroZ);
//			uart_putstr(UART_4, g_TxData);

#if 0
			// 通过串口输出七电感数据
			sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			 (uint16)normalized_data[SENSOR_HL], 
			 (uint16)normalized_data[SENSOR_VL], 
			 (uint16)normalized_data[SENSOR_HML], 
			 (uint16)normalized_data[SENSOR_HC],
			 (uint16)normalized_data[SENSOR_HMR], 
			 (uint16)normalized_data[SENSOR_VR], 
			 (uint16)normalized_data[SENSOR_HR], 
			  position,
			 (uint16)signal_strength_value,
			  track_type,
			  track_route,
			  track_route_status,
			  g_intencoderL,
			  g_intencoderR);
			 uart_putstr(UART_4, g_TxData);
#endif
		}
		
#if 0
		// 处理DMA ADC数据
		if(g_adc_dma_completed_flag)
		{
			process_adc_dma_data(); // 使用DMA数据更新 result[]		

			//读取七电感ADC值（用于调试）
			value[SENSOR_HL] =  result[SENSOR_HL];
			value[SENSOR_VL] =  result[SENSOR_VL];
			value[SENSOR_HML] =  result[SENSOR_HML];
			value[SENSOR_HC] =  result[SENSOR_HC];
			value[SENSOR_HMR] =  result[SENSOR_HMR];
			value[SENSOR_VR] =  result[SENSOR_VR];
			value[SENSOR_HR] =  result[SENSOR_HR];

			// 通过串口输出七电感原始数据
			sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d\n",
						value[SENSOR_HL], 
						value[SENSOR_VL], 
						value[SENSOR_HML], 
						value[SENSOR_HC], 
						value[SENSOR_HMR],
						value[SENSOR_VR],
			value[SENSOR_HR]);
			uart_putstr(UART_4, g_TxData);

			delay_ms(5);
		}

		// sprintf(g_TxData,"error1\n");
		// uart_putstr(UART_4, g_TxData);
		// delay_ms(1000);
#else
		// 获取滤波后的ADC数据		
		// mid_filter();      // 使用中位值滤波获取电感数据 (此通路已不再可用，因get_adc()函数已被移除)
		// sprintf(g_TxData,"error2\n");
		// uart_putstr(UART_4, g_TxData);
		// delay_ms(1000);
#endif


		// // 对DMA获取的数据进行进一步滤波
		// mid_filter();          // 内部会调用 average_filter() 并进行中位值滤波，更新 result[]

		// // 归一化电感数组
		// normalize_sensors();
		
		// // 计算位置偏差
		// position = calculate_position_improved();
		
		//检查电磁保护
		// protection_flag = check_electromagnetic_protection();

		// if(protection_flag)
		// {
		// 	// 触发保护，停车
		// 	// 这里需要添加控制电机停止的代码
			
		// 	// 显示保护触发信息
		// 	ips114_showstr_simspi(0, 7, "Protection: Out of Track!");
			
		// 	// 永久停车或等待重置
		// 	while(1)                                                               
		// 	{
		// 		delay_ms(100);
		// 	}
		// }
		
		// 显示电磁传感器数据
//		display_electromagnetic_data();

		/*调试功能*/
#if 0
		 //读取七电感ADC值（用于调试）
		value[SENSOR_HL] =  result[SENSOR_HL];
		value[SENSOR_VL] =  result[SENSOR_VL];
		value[SENSOR_HML] =  result[SENSOR_HML];
		value[SENSOR_HC] =  result[SENSOR_HC];
		value[SENSOR_HMR] =  result[SENSOR_HMR];
		value[SENSOR_VR] =  result[SENSOR_VR];
		value[SENSOR_HR] =  result[SENSOR_HR];

		 // 通过串口输出七电感原始数据
		  sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d\n",
					value[SENSOR_HL], 
					value[SENSOR_VL], 
					value[SENSOR_HML], 
					value[SENSOR_HC], 
					value[SENSOR_HMR],
					value[SENSOR_VR],
          value[SENSOR_HR]);
		  uart_putstr(UART_4, g_TxData);

		  delay_ms(5);
#endif	

	}	
}



