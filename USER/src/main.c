#include "headfile.h"

void main(void)
{
	int state = 5;
	
	board_init();			
	electromagnetic_init();  // 初始化电磁传感器
	ips114_init_simspi();					
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_4);
	pit_timer_ms(TIM_2, 10);
	pit_timer_ms(TIM_1, 10);
	motor_init();
	encoder_init();
	state = imu963ra_init();
	Kalman_Init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);

	delay_ms(100); // 延时等待系统稳定
	ips114_clear_simspi(WHITE);									//清屏

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
//			if (g_SpeedPoint == 200)
//			{
//				g_SpeedPoint = 500;
//			}
//			else if (g_SpeedPoint == 500)
//			{
//				g_SpeedPoint = 200;
//			}
			
			flag = 0;
		}

		/* 按键处理 */
		key_task();

//		sprintf(g_TxData, "%d,%d\n",position,0);
//		uart_putstr(UART_4, g_TxData);
		
		sprintf(g_TxData, "%d,%d,%d\n", g_EncoderLeft, g_EncoderRight, g_SpeedPoint);
		uart_putstr(UART_4, g_TxData);

		// 获取滤波后的ADC数据
		//average_filter();  // 使用递推均值滤波获取电感数据
		mid_filter();      // 使用中位值滤波获取电感数据

//		// 归一化电感数据
		normalize_sensors();
//		
		// 计算位置偏差
		position = calculate_position_improved();
		

		// sprintf(g_TxData, "%d,%d,%d,%d,%d\n", (uint16)result[SENSOR_L], (uint16)result[SENSOR_LM], (uint16)result[SENSOR_RM], (uint16)result[SENSOR_R], position);
//		sprintf(g_TxData, "%d,%d,%d,%d,%d\n", (uint16)normalized_data[SENSOR_L], (uint16)normalized_data[SENSOR_LM], (uint16)normalized_data[SENSOR_RM], (uint16)normalized_data[SENSOR_R], position);
//		uart_putstr(UART_4, g_TxData);

		//检查电磁保护
		// protection_flag = check_electromagnetic_protection();
		
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

//		delay_ms(5);

		// ips114_showstr_simspi(0,0,"L:");   
		// delay_ms(500);
		// ips114_clear_simspi(RED);									
		// delay_ms(500);
		// ips114_showstr_simspi(0,0,"L:");   
	}	
}



