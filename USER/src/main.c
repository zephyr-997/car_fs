#include "headfile.h"

void main(void)
{
	int state = 5;
	uint16 sum_value = 0; //
	uint16 value[7] = {0}; //调试用数组
	
	board_init();			
	electromagnetic_init();  // 初始化电磁传感器
	
	// ips114_init_simspi();					
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_4);
	
	pit_timer_ms(TIM_1, 10);
	pit_timer_ms(TIM_2, 5);
	
	motor_init();
	encoder_init();
	
	//state = imu963ra_init();
	//Kalman_Init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);
	
	// ips114_clear_simspi(WHITE);	 //清屏
	delay_ms(100); // 延时等待系统稳定
	
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
//				g_SpeedPoint = 50;
//			}
//			else if (g_SpeedPoint == 50)
//			{
//				g_SpeedPoint = 20;
//			}
			
			flag = 0;
		}
		
		/* 按键处理 */
		key_task();

		if (uartSendFlag == 1)
		{
			// sprintf(g_TxData,"%d,%d,%d,%d,%d,%d,%d,%f\n",g_LeftPoint,g_EncoderLeft,g_RightPoint,g_EncoderRight,g_DutyLeft,left_pid,position,turn_pid);
			// uart_putstr(UART_4, g_TxData);
			
			// sprintf(g_TxData, "%f,%f\n",Gyro_Z,filtered_GyroZ);
			// uart_putstr(UART_4, g_TxData);
			
			// 通过串口输出七电感数据
			sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			 (uint16)normalized_data[SENSOR_HL], 
			 (uint16)normalized_data[SENSOR_VL], 
			 (uint16)normalized_data[SENSOR_HML], 
			 (uint16)normalized_data[SENSOR_HC],
			 (uint16)normalized_data[SENSOR_HMR], 
			 (uint16)normalized_data[SENSOR_VR], 
			 (uint16)normalized_data[SENSOR_HR], 
			  position,
			 //(uint16)signal_strength_value,
			  track_type,
			  track_route,
			  track_route_status); 
			uart_putstr(UART_4, g_TxData);
			
			
		}
		
		// 获取滤波后的ADC数据
		//average_filter();  // 使用递推均值滤波获取电感数据
		mid_filter();      // 使用中位值滤波获取电感数据

		// 归一化电感数据
		normalize_sensors();
		
		// 计算位置偏差
		position = calculate_position_improved();
		

		// 计算所有电感值的总和
//		sum_value = (uint16)normalized_data[SENSOR_HL] + (uint16)normalized_data[SENSOR_VL] + 
//		            (uint16)normalized_data[SENSOR_HML] + (uint16)normalized_data[SENSOR_HC] + 
//		            (uint16)normalized_data[SENSOR_HMR] + (uint16)normalized_data[SENSOR_VR] + 
//		            (uint16)normalized_data[SENSOR_HR];


		//检查电磁保护	
		// protection_flag = check_electromagnetic_protection();
		// check_electromagnetic_protection();

		// if(protection_flag)
		// {
		// 	// 触发保护，停车
		// 	// 这里需要添加控制电机停止的代码
			
		// 	// 显示保护触发信息
		// 	ips114_showstr_simspi(0, 7, "Protection: Out of Track!");
			
		// 	// 永久停止或等待重置
		// 	while(1)                                                               
		// 	{
		// 		// 可以添加重置逻辑，例如按键检测
		// 		// 如果需要重新启动，可以在这里添加条件
		// 		delay_ms(100);
		// 	}
		// }
		
		// 显示电磁传感器数据
//		display_electromagnetic_data();

		/*调试功能*/

		// 读取七电感ADC值（用于调试）
		// value[0] = adc_once(ADC_HL,  ADC_10BIT);
		// value[1] = adc_once(ADC_VL,  ADC_10BIT);
		// value[2] = adc_once(ADC_HML, ADC_10BIT);
		// value[3] = adc_once(ADC_HC,  ADC_10BIT); 
		// value[4] = adc_once(ADC_HMR, ADC_10BIT);
		// value[5] = adc_once(ADC_VR,  ADC_10BIT);
		// value[6] = adc_once(ADC_HR,  ADC_10BIT);	

		// // 通过串口输出七电感原始数据
		// sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d\n",
		//  value[0], 
		//  value[1], 
		//  value[2], 
		//  value[3], 
		//  value[4],
		//  value[5],
        //  value[6]);
		//  uart_putstr(UART_4, g_TxData);

		// delay_ms(5);  
	}	
}



