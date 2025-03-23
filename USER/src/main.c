#include "headfile.h"


#define ADC_L   ADC_P00  // 左水平电感
#define ADC_LM  ADC_P01  // 左垂直电感
#define ADC_RM  ADC_P05  // 右垂直电感
#define ADC_R   ADC_P06  // 右水平电感

// 原始ADC数据
uint16 data_L, data_LM, data_RM, data_R;

// 滤波后数据
uint16 filtered_L, filtered_LM, filtered_RM, filtered_R;

// 归一化数据
float normalized_L, normalized_LM, normalized_RM, normalized_R;

// 存储每个电感的最大最小值，用于动态校准
uint16 min_L = 0xFFFF, min_LM = 0xFFFF, min_RM = 0xFFFF, min_R = 0xFFFF;
uint16 max_L = 0, max_LM = 0, max_RM = 0, max_R = 0;

// 电感位置计算相关变量
int16 ad_sum;  
int16 ad_diff;
int16 position;  

// 电磁保护逻辑变量
uint8 protection_flag = 0;  // 全局变量，保护标志

// 函数声明
uint16 adc_mean_filter(ADCN_enum adcn, uint8 count);// 均值滤波函数，修复之前的实现错误
void update_min_max_values(void);  // 更新每个电感的最大最小值	
void normalize_sensors(void);  // 归一化电感数据
int16 calculate_position(void);  // 计算位置
uint8 check_electromagnetic_protection(void);  // 电磁保护逻辑

void main()
{
	board_init();			
	adc_init(ADC_L, 0);						
	adc_init(ADC_LM, 0);						
	adc_init(ADC_RM, 0);
	adc_init(ADC_R, 0);						
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
			ips114_showstr(0, 6, "Protection: Out of Track!");
			
			// 永久停止或等待重置
			while(1)
			{
				// 可以添加重置逻辑，例如按键检测
				// 如果需要重新启动，可以在这里添加条件
				delay_ms(100);
			}
		}
		
		// 显示数据（可以根据需要取消注释）
		ips114_showstr(0,0,"L:");	
		ips114_showuint16(3*8, 0, filtered_L);
		ips114_showstr(9*8,0,"N:");
		ips114_showfloat(11*8, 0, normalized_L, 2, 2);
		
		ips114_showstr(0,1,"LM:");	
		ips114_showuint16(3*8, 1, filtered_LM);
		ips114_showstr(9*8,1,"N:");
		ips114_showfloat(11*8, 1, normalized_LM, 2, 2);
		
		ips114_showstr(0,2,"RM:");	
		ips114_showuint16(3*8, 2, filtered_RM);
		ips114_showstr(9*8,2,"N:");
		ips114_showfloat(11*8, 2, normalized_RM, 2, 2);
		
		ips114_showstr(0,3,"R:");	
		ips114_showuint16(3*8, 3, filtered_R);
		ips114_showstr(9*8,3,"N:");
		ips114_showfloat(11*8, 3, normalized_R, 2, 2);
		
		ips114_showstr(0,4,"Pos:");
		ips114_showint16(5*8, 4, position);
		
		delay_ms(100);
	}	
}

// 均值滤波函数，修复之前的实现错误
uint16 adc_mean_filter(ADCN_enum adcn, uint8 count)
{
	uint8 i = 0;  // 使用uint8代替uint16，因为循环次数很小
	uint32 sum = 0;  // 使用uint32避免溢出
	uint16 adc_value = 0;
	
	// 进行多次采样并累加
	for (i = 0; i < count; i++)
	{
		adc_value = adc_once(adcn, ADC_10BIT);
		sum += adc_value;  // 累加读取值
		// 添加短暂延时可以提高采样稳定性
		delay_us(5);
	}
	
	return (uint16)(sum / count);  // 返回均值
}

// 更新每个电感的最大最小值
void update_min_max_values(void)
{
	// 更新最小值
	if(filtered_L < min_L && filtered_L > 10) min_L = filtered_L;
	if(filtered_LM < min_LM && filtered_LM > 10) min_LM = filtered_LM;
	if(filtered_RM < min_RM && filtered_RM > 10) min_RM = filtered_RM;
	if(filtered_R < min_R && filtered_R > 10) min_R = filtered_R;
	
	// 更新最大值
	if(filtered_L > max_L) max_L = filtered_L;
	if(filtered_LM > max_LM) max_LM = filtered_LM;
	if(filtered_RM > max_RM) max_RM = filtered_RM;
	if(filtered_R > max_R) max_R = filtered_R;
}

// 归一化电感数据（使用整数计算，避免浮点运算）
void normalize_sensors(void)
{
	// 防止除以0，确保最大值和最小值有差异
	// 使用1000作为放大系数，避免使用浮点数
	if(max_L > min_L) 
		normalized_L = (float)((filtered_L - min_L) * 1000) / ((max_L - min_L) * 1000);
	else 
		normalized_L = 0;
		
	if(max_LM > min_LM) 
		normalized_LM = (float)((filtered_LM - min_LM) * 1000) / ((max_LM - min_LM) * 1000);
	else 
		normalized_LM = 0;
		
	if(max_RM > min_RM) 
		normalized_RM = (float)((filtered_RM - min_RM) * 1000) / ((max_RM - min_RM) * 1000);
	else 
		normalized_RM = 0;
		
	if(max_R > min_R) 
		normalized_R = (float)((filtered_R - min_R) * 1000) / ((max_R - min_R) * 1000);
	else 
		normalized_R = 0;
	
	// 限制范围在0-1之间
	if(normalized_L > 1.0f) normalized_L = 1.0f;
	if(normalized_L < 0.0f) normalized_L = 0.0f;
	
	if(normalized_LM > 1.0f) normalized_LM = 1.0f;
	if(normalized_LM < 0.0f) normalized_LM = 0.0f;
	
	if(normalized_RM > 1.0f) normalized_RM = 1.0f;
	if(normalized_RM < 0.0f) normalized_RM = 0.0f;
	
	if(normalized_R > 1.0f) normalized_R = 1.0f;
	if(normalized_R < 0.0f) normalized_R = 0.0f;
}

// 计算位置（使用差比和加权平均方法）
int16 calculate_position(void)
{
    // 在函数开始处声明所有变量
    float weight_sum = 0;
    float weighted_sum = 0;
    float diff_L_R = 0;
    float diff_LM_RM = 0;
    float sum_L_R = 0;
    float sum_LM_RM = 0;
    float ratio_L_R = 0;
    float ratio_LM_RM = 0;
    int16 pos = 0;
    
    // 计算水平电感的差值和和值
    diff_L_R = normalized_L - normalized_R;
    sum_L_R = normalized_L + normalized_R;
    
    // 计算垂直电感的差值和和值
    diff_LM_RM = normalized_LM - normalized_RM;
    sum_LM_RM = normalized_LM + normalized_RM;
    
    // 计算差比和，避免除以0
    if(sum_L_R > 0.01f)
        ratio_L_R = diff_L_R / sum_L_R;
    else
        ratio_L_R = 0;
        
    if(sum_LM_RM > 0.01f)
        ratio_LM_RM = diff_LM_RM / sum_LM_RM;
    else
        ratio_LM_RM = 0;
    
    // 差比和加权，水平电感权重大，垂直电感权重小
    weighted_sum = ratio_L_R * 100 * 0.7f + ratio_LM_RM * 100 * 0.3f;
    
    // 限制范围在-100到100之间
    if(weighted_sum > 100) weighted_sum = 100;
    if(weighted_sum < -100) weighted_sum = -100;
    
    pos = (int16)weighted_sum;
    
    return pos;
}

// 电磁保护逻辑函数实现
uint8 check_electromagnetic_protection(void)
{
    // 在函数开始处声明所有变量
    uint8 is_out_of_track = 0;// 是否脱离赛道标志
    uint16 sum_value = 0;    // 计算所有电感的和值
    uint16 threshold = 100;  // 阈值，需要根据实际情况调整
    static uint8 out_of_track_count = 0; // 脱离赛道计数器
    static uint8 protection_triggered = 0; // 保护触发标志
    
    // 计算所有电感的和值
    sum_value = filtered_L + filtered_LM + filtered_RM + filtered_R;
    
    // 判断是否脱离赛道的条件
    // 1. 所有电感值总和过小，说明可能脱离赛道
    if(sum_value < threshold)
    {
        is_out_of_track = 1;
    }
    
    // 2. 归一化后的值都很小，说明可能脱离赛道
    if(normalized_L < 0.05f && normalized_LM < 0.05f && 
       normalized_RM < 0.05f && normalized_R < 0.05f)
    {
        is_out_of_track = 1;
    }
    
    // 3. 位置偏差过大，说明可能偏离赛道太多
    if(position < -90 || position > 90)
    {
        // 只有当电感值总和也较小时才判断为出赛道
        if(sum_value < threshold * 2)
        {
            is_out_of_track = 1;
        }
    }
    
    // 连续检测逻辑，防止偶然的低值导致误判
    if(is_out_of_track)
    {
        out_of_track_count++;
        if(out_of_track_count >= 5)  // 连续5次检测到脱离赛道才触发保护
        {
            protection_triggered = 1;
        }
    }
    else
    {
        // 如果检测正常，则计数器减少但不低于0
        if(out_of_track_count > 0)
            out_of_track_count--;
    }
    
    return protection_triggered;
}



