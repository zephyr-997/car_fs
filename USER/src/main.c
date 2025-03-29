#include "headfile.h"

void main()
{
	board_init();			
	electromagnetic_init();  // 初始化电磁传感器
	ips114_init();					
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_4);
	pit_timer_ms(TIM_2, 10);
	motor_init();
	encoder_init();
	// imu963ra_init();
	// Kalman_Init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);
	
    while(1)
	{
		
		//串口接收
		if(g_RxPointer != 0)
		{
			int temp = g_RxPointer;
			delay_ms(4);
			if(temp == g_RxPointer)
			{
				uart4_interrupt_callback();
			}
		}
		if (flag == 1)
		{
			flag = 0;
		}		

		sprintf(g_TxData, "%f,%f,%f,%d,%d\n",Gyro_Z, filtered_GyroZ, g_IMU693Point, g_EncoderLeft, g_EncoderRight);
		uart_putstr(UART_4, g_TxData);


		// 获取滤波后的ADC数据
		filtered_L  = adc_mean_filter(ADC_L, 5);  
		filtered_LM = adc_mean_filter(ADC_LM, 5); 	  
		filtered_RM = adc_mean_filter(ADC_RM, 5);		
		filtered_R  = adc_mean_filter(ADC_R, 5);	

		// 更新最大最小值
		update_min_max_values();
		
		// 归一化电感数据
		normalize_sensors();
		
		// 计算位置偏差
		position = calculate_position();
		
		// 检查电磁保护
		//protection_flag = check_electromagnetic_protection();
		
		// if(protection_flag)
		// {
		// 	// 触发保护，停车
		// 	// 这里需要添加控制电机停止的代码
			
		// 	// 显示保护触发信息
		// 	ips114_showstr(0, 7, "Protection: Out of Track!");
			
		// 	// 永久停止或等待重置
		// 	while(1)
		// 	{
		// 		// 可以添加重置逻辑，例如按键检测
		// 		// 如果需要重新启动，可以在这里添加条件
		// 		delay_ms(100);
		// 	}
		// }
		
		// 显示电磁传感器数据
		display_electromagnetic_data();
		
		delay_ms(100);
	}	
}



