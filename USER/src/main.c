#include "headfile.h"


void main()
{
	board_init();			
	electromagnetic_init();  // 初始化电磁传感器
	ips114_init();						
	
	
	
    while(1)
	{
		
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
		protection_flag = check_electromagnetic_protection();
		
		if(protection_flag)
		{
			// 触发保护，停车
			// 这里需要添加控制电机停止的代码，例如：
			// motor_set(MOTOR_L, 0);
			// motor_set(MOTOR_R, 0);
			
			// 显示保护触发信息
			ips114_showstr(0, 7, "Protection: Out of Track!");
			
			// 永久停止或等待重置
			while(1)
			{
				// 可以添加重置逻辑，例如按键检测
				// 如果需要重新启动，可以在这里添加条件
				delay_ms(100);
			}
		}
		
		// 显示电磁传感器数据
		display_electromagnetic_data();
		
		delay_ms(100);
	}	
}



