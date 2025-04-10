#include "electromagnetic_tracking.h"
#include "headfile.h"
#include "common.h"

// 滤波后数据 - 使用二维数组形式
// 第一维表示电感编号：0-L, 1-LM, 2-RM, 3-R
// 第二维保留，可用于存储历史数据
#define SENSOR_COUNT 4   //电感个数
#define HISTORY_COUNT 5  //滤波次数

uint16 adc_fliter_data[SENSOR_COUNT][HISTORY_COUNT] = {0}; //滤波后的值
float result[SENSOR_COUNT] = {0};		//电存储每个电感滤波后的最终结果值（尚未归一化），是连接滤波处理和归一化处理的中间变量
uint16 sum[SENSOR_COUNT][HISTORY_COUNT] = {0};      	//累加的和

// 递推均值滤波相关参数
uint16 times = HISTORY_COUNT;  // 滤波次数
uint16 i_num = SENSOR_COUNT;  // 电感数量

// 归一化数据 - 改为数组形式
float normalized_data[SENSOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};  // 归一化后的电感数据数组

// 存储每个电感的最大最小值，用于动态校准 - 改为数组形式
uint16 min_value[SENSOR_COUNT] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};  // 每个电感的最小值
uint16 max_value[SENSOR_COUNT] = {0, 0, 0, 0};  // 每个电感的最大值

// 电感位置计算相关变量
int16 position = 0;

// 电磁保护逻辑变量,0表示未保护，1表示保护
uint8 protection_flag = 0;



//-----------------------------------------------------------------------------
// @brief  	电磁传感器初始化
// @param   无
// @return  无
// @author  ZP
// Sample usage: electromagnetic_init();
//-----------------------------------------------------------------------------
void electromagnetic_init(void)
{
   uint8 i = 0, j = 0;

   adc_init(ADC_L, 0);
   adc_init(ADC_LM, 0);
   adc_init(ADC_RM, 0);
   adc_init(ADC_R, 0);
   
   // 初始化二维数组
   for(i = 0; i < SENSOR_COUNT; i++)
   {
       for(j = 0; j < HISTORY_COUNT; j++)
       {
           adc_fliter_data[i][j] = 0;
           sum[i][j] = 0;
       }
       normalized_data[i] = 0.0f;  // 初始化归一化数据数组
   }
}

//-----------------------------------------------------------------------------
// @brief  	得到adc的值
// @param   测的ADC/电感序号
// @return  测出的adc值
// @author  ZP
// Sample usage: get_adc(1)
//-----------------------------------------------------------------------------
uint16 get_adc(uint16 i)
{
	switch(i){
		case 0:
			return adc_once(ADC_L, ADC_10BIT);  //ADC_10BIT是电磁寻迹最佳分辨率
		case 1:
			return adc_once(ADC_LM, ADC_10BIT);
		case 2:
			return adc_once(ADC_RM, ADC_10BIT);
		case 3:
			return adc_once(ADC_R, ADC_10BIT);
		default:
			return 0;
	}
}

//-----------------------------------------------------------------------------
// @brief  	递推均值滤波
// @param   无
// @return  无
// @author  zp
// Sample usage: average_filter();
//-----------------------------------------------------------------------------


void average_filter(void)
{
    static uint16 filter_index = 0;  // 递推次数计数器
    static uint8 is_initialized = 0; // 初始化标志,只在第一次调用时进行多次采样,后续调用时使用真正的递推算法   
    uint16 a = 0, b = 0;
    
    // 检查是否需要初始化
    if (!is_initialized)
    {
        // 重置累加器
        for(a = 0; a < i_num; a++)
        {
            sum[a][0] = 0;
        }
        
        // 前几次采集，累积足够的数据
        for(filter_index = 0; filter_index < times; filter_index++)
        {
            for(b = 0; b < i_num; b++)
            {
                sum[b][0] += get_adc(b);  // 采集一次ADC并累加
            }
            delay_us(5); // 添加短暂延时提高采样稳定性
        }
        
        // 计算初始平均值
        for(a = 0; a < i_num; a++)
        {
            adc_fliter_data[a][0] = sum[a][0] / times;  // 求平均
            result[a] = adc_fliter_data[a][0];
        }
        
        is_initialized = 1; // 标记初始化完成
    }
    else  // 已初始化，执行递推均值滤波
    {
        for(a = 0; a < i_num; a++)
        {
            // 递推均值滤波核心算法：减去平均值，加上新值，重新计算平均值
            sum[a][0] -= (sum[a][0] / times);         // 每次去除平均值的贡献
            sum[a][0] += get_adc(a);              // 加上新值
            adc_fliter_data[a][0] = (sum[a][0] / times);  // 求新的平均值
            result[a] = adc_fliter_data[a][0];      // 保存结果
        }
    }
}


//-----------------------------------------------------------------------------
// @brief  	中位值滤波，将每个电感的中位数作为结果
// @param   无
// @return  无
// @author  ZP
// Sample usage: mid_filter();
//-----------------------------------------------------------------------------
static uint8 mid_initialized = 0;  // 中位值滤波初始化标志
static uint16 sample_count = 0;    // 采样计数器

void mid_filter(void)
{
    uint16 temp = 0, a = 0, t = 0;
    uint16 mid_index = 0;  //中位数
	uint16 i = 0; //用于循环
    // 创建临时数组用于排序，避免修改原始数据
    uint16 sort_array[HISTORY_COUNT];  // 使用宏定义的常量而不是变量
	
    // 调用均值滤波获取新的采样值
    average_filter();
    
    // 如果尚未初始化完成
    if (!mid_initialized)
    {
        // 将当前滤波结果存入历史数组
        for(a = 0; a < i_num; a++)
        {
            adc_fliter_data[a][sample_count] = adc_fliter_data[a][0];
            result[a] = adc_fliter_data[a][0];
        }
        
        sample_count++;
        
        // 当采集到足够样本时，标记初始化完成
        if (sample_count >= times)
        {
            mid_initialized = 1;
            sample_count = 0;  // 重置计数器用于循环缓冲
        }
    }
    else  // 已初始化，执行中位值滤波
    {
        // 更新历史数据数组
        for(a = 0; a < i_num; a++)
        {
            adc_fliter_data[a][sample_count] = adc_fliter_data[a][0];
        }
        
        // 更新循环缓冲区索引
        sample_count = (sample_count + 1) % times;
        
        // 对每个电感通道进行处理
        for(a = 0; a < i_num; a++)
        {
            for(t = 0; t < times; t++)
            {
                sort_array[t] = adc_fliter_data[a][t];
            }
            
            // 冒泡排序
            for(i = 0; i < times-1; i++)
            {
                for(t = 0; t < times-i-1; t++)
                {
                    if(sort_array[t] > sort_array[t+1])
                    {
                        temp = sort_array[t];
                        sort_array[t] = sort_array[t+1];
                        sort_array[t+1] = temp;
                    }
                }
            }
            
            // 计算中位数索引
            mid_index = times / 2;  // 5 / 2 = 2  ,sort_array[2]是第三个数即中位数
            
            // 取中位数作为结果
            result[a] = sort_array[mid_index];
        }
    }
}


//-----------------------------------------------------------------------------
// @brief  	更新每个电感的最大最小值，用于动态校准
// @param   无
// @return  无
// @author  ZP
// Sample usage: update_min_max_values();
//-----------------------------------------------------------------------------
void update_min_max_values(void)
{
    uint8 i;
    static uint16 update_counter = 0;
    
    // 定期轻微衰减最大最小值，使系统能适应环境变化
    update_counter++;
    if(update_counter >= 1000)  // 每1000次调用执行一次衰减
    {
        update_counter = 0;
        
        // 最小值略微增加，最大值略微减少，形成缓慢衰减
        for(i = 0; i < SENSOR_COUNT; i++)
        {
            // 最小值向上衰减（增加1%）
            min_value[i] += min_value[i] / 100;
            
            // 最大值向下衰减（减少1%）
            if(max_value[i] > min_value[i])  // 确保最大值始终大于最小值
                max_value[i] -= max_value[i] / 100;
        }
    }
    
    // 更新每个电感的最小值和最大值
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        // 异常值检测 - 如果读数异常大或异常小，可能是传感器故障，不更新
        if(result[i] > 1000 || result[i] < 5)
            continue;
            
        // 更新最小值（忽略过小的值，可能是噪声）
        if(result[i] < min_value[i] && result[i] > 10) 
            min_value[i] = result[i];
        
        // 更新最大值
        if(result[i] > max_value[i]) 
            max_value[i] = result[i];
    }
    
    // 确保最大最小值之间有足够差距，避免除以接近0的值
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        if(max_value[i] - min_value[i] < 20)
        {
            // 如果差距太小，强制设置一个合理差距
            max_value[i] = min_value[i] + 20;
        }
    }
}

//-----------------------------------------------------------------------------
// @brief  	归一化电感数据
// @param   无
// @return  无
// @author  ZP
// Sample usage: normalize_sensors();
//-----------------------------------------------------------------------------
void normalize_sensors(void)
{
    uint8 i;
    // 可选：对归一化后的数据进行平滑处理，减少抖动
    static float last_normalized[SENSOR_COUNT] = {0};
    // 平滑因子，可调整：值越大，响应越快但抖动越明显，值越小，响应越慢但更平稳
    float smooth_factor = 0.7f; // 建议在0.6-0.8范围内调整，根据小车实际表现微调    // 首先更新最大最小值

    update_min_max_values();
    
    // 对每个电感进行归一化处理
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        // 检查最大最小值差异是否足够大，防止除以接近0的值
        if(max_value[i] - min_value[i] > 20) 
        {
            // 标准线性归一化，将值映射到0-100范围（乘以100方便后续使用）
            normalized_data[i] = (float)(result[i] - min_value[i]) / (max_value[i] - min_value[i]) * 100.0f;
            
            // 可选：使用平方根非线性映射，增强小信号响应(如果发现小车对小偏差反应不敏感，可以取消平方根映射的注释)
            // normalized_data[i] = sqrtf(normalized_data[i] / 100.0f) * 100.0f;
        }
        else 
        {
            // 如果最大最小值差异太小，可能是传感器故障或未正确初始化
            // 使用原始值的相对比例作为替代，也乘以100保持一致性
            normalized_data[i] = (float)result[i] / 10.0f;  // 假设ADC最大值为1000，归一化到0-100
        }
        
        // 限制范围在0-100之间
        if(normalized_data[i] > 100.0f) normalized_data[i] = 100.0f;
        if(normalized_data[i] < 0.0f) normalized_data[i] = 0.0f;
    }
    
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        // 低通滤波平滑处理
        normalized_data[i] = normalized_data[i] * smooth_factor + 
                            last_normalized[i] * (1.0f - smooth_factor);
        
        // 保存当前值用于下次平滑
        last_normalized[i] = normalized_data[i];
    }
}

//-----------------------------------------------------------------------------
// @brief  	改进版计算位置（使用自适应差比和加权方法）
// @param   无
// @return  计算得到的位置值，范围-100到100
// @author  ZP
// Sample usage: position = calculate_position_improved();
//-----------------------------------------------------------------------------
int16 calculate_position_improved(void)
{
    // 在函数开始处声明所有变量
    float weight_L_R = 0.7f;   // 左右电感权重
    float weight_LM_RM = 0.3f; // 左中右中电感权重
    float diff_L_R = 0;        // 左右水平电感差值
    float diff_LM_RM = 0;      // 左右中间电感差值
    float sum_L_R = 0;         // 左右水平电感和值
    float sum_LM_RM = 0;       // 左右中间电感和值
    float ratio_L_R = 0;       // 左右电感差比和
    float ratio_LM_RM = 0;     // 左中右中电感差比和
    float signal_strength = 0; // 信号强度指标
    static int16 last_pos = 0; // 上一次位置值，用于滤波
    int16 pos = 0;             // 当前计算得到的位置值
    float filter_param = 0.7f; // 滤波系数，可调
    uint8 track_type = 0;      // 赛道类型：0-普通，1-十字，2-环岛，3-坡道
    
    // 计算各对电感的差值和和值
    diff_L_R = normalized_data[SENSOR_L] - normalized_data[SENSOR_R];
    sum_L_R = normalized_data[SENSOR_L] + normalized_data[SENSOR_R];
    
    diff_LM_RM = normalized_data[SENSOR_LM] - normalized_data[SENSOR_RM];
    sum_LM_RM = normalized_data[SENSOR_LM] + normalized_data[SENSOR_RM];
    
    // 计算信号强度指标
    signal_strength = (sum_L_R + sum_LM_RM) / 4.0f;
    
    // 计算差比和，避免除以0
    if(sum_L_R > 1.0f)
        ratio_L_R = diff_L_R / sum_L_R;
    else
        ratio_L_R = 0;
        
    if(sum_LM_RM > 1.0f)
        ratio_LM_RM = diff_LM_RM / sum_LM_RM;
    else
        ratio_LM_RM = 0;
    
    // 赛道类型识别
    // 1. 十字路口特征：中间电感值大，两侧电感值小
    if(normalized_data[SENSOR_LM] > 60.0f && normalized_data[SENSOR_RM] > 60.0f && 
       normalized_data[SENSOR_L] < 30.0f && normalized_data[SENSOR_R] < 30.0f)
    {
        track_type = 1; // 十字路口
    }
    // 2. 环岛特征：一侧电感值很大，另一侧很小
    else if((normalized_data[SENSOR_L] > 80.0f && normalized_data[SENSOR_R] < 20.0f) ||
            (normalized_data[SENSOR_R] > 80.0f && normalized_data[SENSOR_L] < 20.0f))
    {
        track_type = 2; // 环岛
    }
    // 3. 坡道特征：所有电感值都较大
    else if(normalized_data[SENSOR_L] > 70.0f && normalized_data[SENSOR_R] > 70.0f &&
            normalized_data[SENSOR_LM] > 70.0f && normalized_data[SENSOR_RM] > 70.0f)
    {
        track_type = 3; // 坡道
    }
    
    // 根据赛道类型和信号强度调整权重
    switch(track_type)
    {
        case 0: // 普通赛道
            // 根据信号强度动态调整权重
            if(signal_strength > 70.0f) // 信号强，可能在直道
            {
                weight_L_R = 0.6f;
                weight_LM_RM = 0.4f;
            }
            else if(signal_strength < 30.0f) // 信号弱，可能在弯道
            {
                weight_L_R = 0.8f;
                weight_LM_RM = 0.2f;
            }
            break;
            
        case 1: // 十字路口
            // 十字路口更依赖中间电感
            weight_L_R = 0.3f;
            weight_LM_RM = 0.7f;
            break;
            
        case 2: // 环岛
            // 环岛更依赖外侧电感
            weight_L_R = 0.9f;
            weight_LM_RM = 0.1f;
            break;
            
        case 3: // 坡道
            // 坡道使用平均权重
            weight_L_R = 0.5f;
            weight_LM_RM = 0.5f;
            break;
    }
    
    // 特殊情况处理：当所有电感值都很小时，可能已经偏离赛道
    if(sum_L_R < 10.0f && sum_LM_RM < 10.0f)
    {
        // 根据上一次位置判断偏离方向
        if(last_pos > 0)
            return 100;  // 向右偏离
        else
            return -100; // 向左偏离
    }
    
    // 差比和加权平均
    pos = (int16)((ratio_L_R * weight_L_R + ratio_LM_RM * weight_LM_RM) * 100.0f);
    
    // 限制范围在-100到100之间
    if(pos > 100) pos = 100;
    if(pos < -100) pos = -100;
    
    // 根据赛道类型调整滤波参数
    if(track_type == 0) // 普通赛道
    {
        // 根据信号强度调整滤波参数
        if(signal_strength > 70.0f) // 信号强，可能在直道
            filter_param = 0.8f; // 响应更快
        else
            filter_param = 0.6f; // 更平滑
    }
    else if(track_type == 1) // 十字路口
    {
        filter_param = 0.5f; // 十字路口需要更平滑的控制
    }
    else if(track_type == 2) // 环岛
    {
        filter_param = 0.9f; // 环岛需要更快的响应
    }
    
    // 添加自适应低通滤波，减少抖动
    pos = (int16)(filter_param * pos + (1-filter_param) * last_pos);
    last_pos = pos;
    
    return pos;
}


//-----------------------------------------------------------------------------
// @brief  	电磁保护逻辑函数
// @param   无
// @return  保护触发标志位，1表示已触发保护
// @author  ZP
// Sample usage: protection_flag = check_electromagnetic_protection();
//-----------------------------------------------------------------------------
uint8 check_electromagnetic_protection(void)
{
    // 在函数开始处声明所有变量
    uint8 is_out_of_track = 0;    // 标记是否脱离赛道的标志位
    uint16 sum_value = 0;         // 所有电感值的总和
    uint16 threshold = 100;       // 阈值，需要根据实际情况调整，判断脱离赛道的电感总和阈值
    static uint8 out_of_track_count = 0;    // 连续检测到脱离赛道的次数计数器
    static uint8 protection_triggered = 0;  // 保护触发标志位，1表示已触发保护
    uint8 i;
    
    // 计算所有电感的和值
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        sum_value += result[i];
    }
    
    // 判断是否脱离赛道的条件
    // 1. 所有电感值总和过小，说明可能脱离赛道
    if(sum_value < threshold)
    {
        is_out_of_track = 1;
    }
    
    // 2. 归一化后的值都很小，说明可能脱离赛道
    if(normalized_data[SENSOR_L] < 5.0f && normalized_data[SENSOR_LM] < 5.0f && 
       normalized_data[SENSOR_RM] < 5.0f && normalized_data[SENSOR_R] < 5.0f)
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

// 显示电磁传感器数据
void display_electromagnetic_data(void)
{
    // 显示原始滤波数据和归一化数据
    ips114_showstr_simspi(0,0,"L:");   
    ips114_showuint16_simspi(3*8, 0, result[SENSOR_L]);
    ips114_showstr_simspi(9*8,0,"N:");
    ips114_showfloat_simspi(11*8, 0, normalized_data[SENSOR_L], 2, 2);
    
    ips114_showstr_simspi(0,1,"LM:");  
    ips114_showuint16_simspi(3*8, 1, result[SENSOR_LM]);
    ips114_showstr_simspi(9*8,1,"N:");
    ips114_showfloat_simspi(11*8, 1, normalized_data[SENSOR_LM], 2, 2);
    
    ips114_showstr_simspi(0,2,"RM:");  
    ips114_showuint16_simspi(3*8, 2, result[SENSOR_RM]);
    ips114_showstr_simspi(9*8,2,"N:");
    ips114_showfloat_simspi(11*8, 2, normalized_data[SENSOR_RM], 2, 2);
    
    ips114_showstr_simspi(0,3,"R:");   
    ips114_showuint16_simspi(3*8, 3, result[SENSOR_R]);
    ips114_showstr_simspi(9*8,3,"N:");
    ips114_showfloat_simspi(11*8, 3, normalized_data[SENSOR_R], 2, 2);
    
    // 显示位置和差比和数据
    ips114_showstr_simspi(0,4,"Pos:");
    ips114_showint16_simspi(5*8, 4, position);
    
} 