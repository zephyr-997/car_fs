#include "electromagnetic_tracking.h"
#include "headfile.h"
#include "common.h"
#include "STC32G_NVIC.h"
#include "zf_uart.h"
#include "zf_delay.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// DMA ADC错误码定义
#define DMA_ADC_ERROR_NONE          0  // 无错误
#define DMA_ADC_ERROR_TIMEOUT       1  // DMA转换超时
#define DMA_ADC_ERROR_INVALID_DATA  2  // 数据异常
#define DMA_ADC_ERROR_CONFLICT      3  // 资源冲突

// DMA ADC缓冲区和标志
//uint16 AdcDmaBuffer[ADC_DMA_USED_CHANNEL_COUNT][ADC_DMA_SAMPLES_PER_CHANNEL_NUM] = {0}; // DMA ADC缓冲区
volatile uint8 g_adc_dma_completed_flag = 0;  // DMA ADC数据就绪标志（统一标志）

unsigned char xdata AdcDmaBuffer[ADC_DMA_USED_CHANNEL_COUNT][ADC_DMA_SAMPLES_PER_CHANNEL_NUM];

// 滤波后数据 - 使用二维数组形式
// 第一维表示电感编号：0-HL, 1-VL, 2-HML, 3-HC, 4-HMR, 5-VR, 6-HR
// 第二维保留，可用于存储历史数据
#define SENSOR_COUNT 7   //电感个数
#define HISTORY_COUNT 5  //滤波次数

// 赛道类型索引，与track_type对应
#define WEIGHT_STRAIGHT    0  // 直道
#define WEIGHT_RIGHT_ANGLE 1  // 直角弯道
#define WEIGHT_CROSS       2  // 十字圆环
#define WEIGHT_ROUNDABOUT  3  // 环岛
#define WEIGHT_SPEED       4  //深度加速 



// 定义电感权重结构体	

// 定义全局权重配置，只保留四种基本元素
TrackWeights track_weights[4] = {
    // 普通直道

    {0.15f, 0.40f, 0.30f, 0.15f, 0.80f, 35, "直道"},
    
    // 直角弯道
    {0.25f, 0.30f, 0.35f, 0.35f, 1.00f, 50, "直角弯道"},
    
    // 十字圆环
    {0.35f, 0.25f, 0.20f, 0.15f, 0.90f, 40, "十字圆环"},
    
    // 环岛
    {0.31f, 0.43f, 0.10f, 0.21f, 1.00f, 40, "环岛"}
};

uint16 adc_fliter_data[SENSOR_COUNT][HISTORY_COUNT] = {0}; //滤波后的值
float result[SENSOR_COUNT] = {0};		//电存储每个电感滤波后的最终结果值（尚未归一化），是连接滤波处理和归一化处理的中间变量
uint16 sum[SENSOR_COUNT][HISTORY_COUNT] = {0};      	//累加的和

// 递推均值滤波相关参数
uint16 times = HISTORY_COUNT;  // 滤波次数
uint16 i_num = SENSOR_COUNT;  // 电感数量

// 归一化数据 - 改为数组形式
float normalized_data[SENSOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // 归一化后的电感数据数组

// 存储每个电感的最大最小值，用于动态校准 - 改为数组形式
// uint16 min_value[SENSOR_COUNT] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};  // 每个电感的最小值
// uint16 max_value[SENSOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};  // 每个电感的最大值
uint16 min_value[SENSOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};  // 每个电感的最小值
uint16 max_value[SENSOR_COUNT] = {910, 930, 920, 730, 920, 930, 910};  // 每个电感的最大值

// 电感位置计算相关变量
float signal_strength_value = 0;   // 信号强度指标
int16 position = 0;
float filter_param = 0.4f;   // 滤波系数，可调 越大越灵敏


// 赛道信息相关标志位
uint8 track_type = 0;         // 赛道类型：0-普通，1-直角弯道，2-十字圆环，3-环岛
uint8 track_type_last = 0;         // 赛道类型：0-普通，1-直角弯道，2-十字圆环，3-环岛

uint8 track_type_zj = 0;	  //1-左直角，2-右直角
uint8 track_route = 0; 		  //1-右环，2-左环
uint8 track_route_status = 0; //1-入环，2-环中，3-出环
uint8 track_ten_flag = 1;	//十字圆环：0表示到计时0.5s再开始判断，1-可以开始判断
uint8 ten_change_flag = 0; //1表示0.5后track_ten_flag=1

uint8 protection_flag = 0;// 电磁保护逻辑变量,0表示未保护，1表示保护


uint8 speed_count = 0;

//-----------------------------------------------------------------------------
// @brief    电磁传感器DMA ADC初始化
// @param    无
// @return   无
// @author   
// Sample usage: electromagnetic_dma_init();
//-----------------------------------------------------------------------------
void electromagnetic_dma_init(void)
{
    ADC_InitTypeDef adc_init_struct;
    DMA_ADC_InitTypeDef dma_adc_init_struct;
    uint8 i = 0, j = 0; // 变量声明移到函数开始处

    uint32 test_channel_num = 0; // 测试通道数

    unsigned char debug_buffer[60];
    
    // 配置ADC参数
    adc_init_struct.ADC_SMPduty = 15;                // ADC 模拟信号采样时间控制, 0~31, 一般大于10
    adc_init_struct.ADC_Speed = ADC_SPEED_2X16T;     // ADC 转换速度, 相对于 SYSCLK/2
    adc_init_struct.ADC_AdjResult = ADC_RIGHT_JUSTIFIED; // ADC 转换结果对齐方式, 右对齐
    adc_init_struct.ADC_CsSetup = 0;                 // ADC 通道选择建立时间控制, 0(默认),1
    adc_init_struct.ADC_CsHold = 1;                  // ADC 通道选择保持时间控制, 0,1(默认),2,3

    // 初始化ADC
    ADC_Inilize(&adc_init_struct);
    ADC_PowerControl(ENABLE);                        // 使能ADC模块
    uart_putstr(UART_4, "ADC initialized and powered on\r\n");

    // 配置DMA ADC参数
    dma_adc_init_struct.DMA_Enable = ENABLE;         // 使能DMA
    
    // 打印DMA通道配置
    test_channel_num = ELECTROMAGNETIC_DMA_CHANNELS;

    sprintf(debug_buffer, "DMA Channels 0x%08LX\r\n", test_channel_num);
    uart_putstr(UART_4, debug_buffer);
    
    dma_adc_init_struct.DMA_Channel = ELECTROMAGNETIC_DMA_CHANNELS; // 使能的通道
    dma_adc_init_struct.DMA_Buffer = (unsigned int)AdcDmaBuffer; // DMA缓冲区地址
    dma_adc_init_struct.DMA_Times = ADC_4_Times;     // 每个通道采样4次，对应ADC_DMA_SAMPLES_PER_CHANNEL_NUM=4

    // 初始化DMA ADC
    DMA_ADC_Inilize(&dma_adc_init_struct);
    uart_putstr(UART_4, "DMA ADC initialized\r\n");
    
    // 使能DMA ADC中断并设置优先级
    NVIC_DMA_ADC_Init(ENABLE, Priority_1, Priority_1);
    uart_putstr(UART_4, "DMA ADC interrupt enabled\r\n");

    // 初始化DMA缓冲区
    for(i = 0; i < ADC_DMA_USED_CHANNEL_COUNT; i++)
    {
        for(j = 0; j < ADC_DMA_SAMPLES_PER_CHANNEL_NUM; j++)
        {
            AdcDmaBuffer[i][j] = 0;
        }
    }

    // 初始化数据数组 (从electromagnetic_init函数移植过来)
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        for(j = 0; j < HISTORY_COUNT; j++)
        {
            adc_fliter_data[i][j] = 0;
            sum[i][j] = 0;
        }
        normalized_data[i] = 0.0f;  // 初始化归一化数据数组
    }
    
    // 确保DMA完成标志初始状态为0
    g_adc_dma_completed_flag = 0;
    
    uart_putstr(UART_4, "DMA ADC initialization complete\r\n");
    
    // 启动第一次DMA ADC转换
    DMA_ADC_TRIG(); 

    uart_putstr(UART_4, "First DMA ADC conversion triggered\r\n");
}

//-----------------------------------------------------------------------------
// @brief    启动DMA ADC转换
// @param    无
// @return   无
// @author   
// Sample usage: start_adc_dma_conversion();
//-----------------------------------------------------------------------------
void start_adc_dma_conversion(void)
{
    // 注意：我们改变了逻辑，无论标志位状态如何都尝试启动转换
    // 如果上一次转换尚未完成，可能会出现非法触发错误，但这将在中断中处理
    
    // 可选：清除可能的错误标志
    // DMA_ADC_STA &= ~0x01; // 清除非法触发错误标志
    
    // 触发DMA ADC转换
    DMA_ADC_TRIG();
}

//-----------------------------------------------------------------------------
// @brief    处理DMA ADC数据
// @param    无
// @return   无
// @author   
// Sample usage: process_adc_dma_data();
//-----------------------------------------------------------------------------
void process_adc_dma_data(void)
{
    uint8 i, j;
    uint32 sum_values;

    // 创建一个临时数组来按 DMA 通道顺序存储平均值
    float dma_ordered_averages[ADC_DMA_USED_CHANNEL_COUNT];
    
    // 检查DMA ADC数据是否就绪
    if(!g_adc_dma_completed_flag)
    {
        return;
    }
    
    // 清除标志位
    g_adc_dma_completed_flag = 0;
    
    // 1. 从 AdcDmaBuffer 计算平均值，并按 DMA 通道顺序存入 dma_ordered_averages
    // DMA 物理存储顺序: HC(ch1), HMR(ch3), VR(ch4), HR(ch8), HML(ch9), VL(ch13), HL(ch14)
    for(i = 0; i < ADC_DMA_USED_CHANNEL_COUNT; i++)
    {
        sum_values = 0;
        for(j = 0; j < ADC_DMA_SAMPLES_PER_CHANNEL_NUM; j++)
        {
            sum_values += AdcDmaBuffer[i][j];
        }
        dma_ordered_averages[i] = (float)sum_values / ADC_DMA_SAMPLES_PER_CHANNEL_NUM;
    }

    // 2. 根据 sensor_type_e 将数据从 dma_ordered_averages 映射到 result
    //    result[SENSOR_HL]  <-- 来自 HL (DMA channel 14, AdcDmaBuffer[6])
    //    result[SENSOR_VL]  <-- 来自 VL (DMA channel 13, AdcDmaBuffer[5])
    //    result[SENSOR_HML] <-- 来自 HML (DMA channel 9, AdcDmaBuffer[4])
    //    result[SENSOR_HC]  <-- 来自 HC (DMA channel 1, AdcDmaBuffer[0])
    //    result[SENSOR_HMR] <-- 来自 HMR (DMA channel 3, AdcDmaBuffer[1])
    //    result[SENSOR_VR]  <-- 来自 VR (DMA channel 4, AdcDmaBuffer[2])
    //    result[SENSOR_HR]  <-- 来自 HR (DMA channel 8, AdcDmaBuffer[3])

    result[SENSOR_HL] = dma_ordered_averages[6]; // HL is the 7th in DMA sequence (index 6)
    result[SENSOR_VL] = dma_ordered_averages[5]; // VL is the 6th in DMA sequence (index 5)
    result[SENSOR_HML]= dma_ordered_averages[4]; // HML is the 5th in DMA sequence (index 4)
    result[SENSOR_HC] = dma_ordered_averages[0]; // HC is the 1st in DMA sequence (index 0)

    result[SENSOR_HMR]= dma_ordered_averages[1]; // HMR is the 2nd in DMA sequence (index 1)
    result[SENSOR_VR] = dma_ordered_averages[2]; // VR is the 3rd in DMA sequence (index 2)
    result[SENSOR_HR] = dma_ordered_averages[3]; // HR is the 4th in DMA sequence (index 3)
    
    // 启动下一次DMA ADC转换
    start_adc_dma_conversion();
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
    static uint8 is_initialized = 0; // 初始化标志
    uint16 a = 0;
    
    // process_adc_dma_data() 应该在此函数之前被调用, 
    // 因此 result[a] 包含最新的DMA平均值。
    
    if (!is_initialized)
    {
        // 初始化 sum[a][0] 使用来自 result[a] 的第一个样本
        // 并假设这个样本在初始化阶段是稳定的。
        for(a = 0; a < i_num; a++)
        {
            sum[a][0] = (uint32)result[a] * times; // 用当前的 result 值 priming sum
            adc_fliter_data[a][0] = result[a];   // 第一个平均值就是当前的 result 值
        }
        is_initialized = 1; // 标记初始化完成
    }
    else  // 已初始化，执行递推均值滤波
    {
        for(a = 0; a < i_num; a++)
        {
            // 递推均值滤波核心算法：减去近似最旧值的贡献，加上新值，重新计算平均值
            // result[a] 此刻是由 process_adc_dma_data() 更新的最新DMA平均值
            sum[a][0] -= (sum[a][0] / times);     // 减去旧的平均值的贡献（近似移除最旧样本）
            sum[a][0] += (uint32)result[a];       // 加上新的样本 (来自DMA的 result[a])
            adc_fliter_data[a][0] = (sum[a][0] / times);  // 计算新的平均值
            result[a] = adc_fliter_data[a][0];    // 用这个更长期的平均值更新 result[a]
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
            
            // 获取中位数
            mid_index = times / 2;
            if(times % 2 == 0)  // 偶数个样本，取中间两个的平均
            {
                result[a] = (sort_array[mid_index-1] + sort_array[mid_index]) / 2;
            }
            else  // 奇数个样本，取中间值
            {
                result[a] = sort_array[mid_index];
            }
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

    // update_min_max_values();
    
    // 对每个电感进行归一化处理
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        // 检查最大最小值差异是否足够大，防止除以接近0的值
        if(max_value[i] - min_value[i] > 20) 
        {
            // 标准线性归一化，将值映射到0-100范围（乘以100方便后续使用）
            normalized_data[i] = (float)(result[i] - min_value[i]) * 100.0f / (max_value[i] - min_value[i]);
            
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
    float weight_outer = 0.15f;   // 外侧电感权重(HL和HR)
    float weight_middle = 0.40f;  // 中间电感权重(HML和HMR)
    float weight_vertical = 0.15f; // 纵向电感权重(VL和VR)
    float weight_center = 0.0f;  // 中心电感权重(HC)(没用上)
    
    float diff_outer = 0;        // 外侧电感差值
    float diff_middle = 0;       // 中间电感差值
    float diff_vertical = 0;     // 纵向电感差值
    
    float sum_outer = 0;         // 外侧电感和值
    float sum_middle = 0;        // 中间电感和值
    float sum_vertical = 0;      // 纵向电感和值
    float center_value = 0;      // 中间电感值
    
    float ratio_outer = 0;       // 外侧电感差比和
    float ratio_middle = 0;      // 中间电感差比和
    float ratio_vertical = 0;    // 纵向电感差比和
    
    float signal_strength = 0;   // 信号强度指标
    static int16 last_pos = 0;   // 上一次位置值，用于滤波
    static int16 very_last_pos = 0;  // 上上次位置值，用于二次滤波
    static int16 very_very_last_pos = 0;  // 上上上次位置值，用于三次滤波
    int16 pos = 0;               // 当前计算得到的位置值
    static int16 max_change_rate = 8; // 允许的最大变化率，越大越灵敏
    int16 position_change = 0;   // 位置变化量
	
	
	
	
	// 位置计算（包含中心电感的贡献）
    // 中心电感越大，位置越接近中心线，这里直接将中心电感作为位置修正因子
    float center_correction = 0;
    
    // 计算各对电感的差值和和值
    diff_outer = normalized_data[SENSOR_HL] - normalized_data[SENSOR_HR];
    sum_outer = normalized_data[SENSOR_HL] + normalized_data[SENSOR_HR];
    
    diff_middle = normalized_data[SENSOR_HML] - normalized_data[SENSOR_HMR];
    sum_middle = normalized_data[SENSOR_HML] + normalized_data[SENSOR_HMR];
    
    diff_vertical = normalized_data[SENSOR_VL] - normalized_data[SENSOR_VR];
    sum_vertical = normalized_data[SENSOR_VL] + normalized_data[SENSOR_VR];
    
    // 获取中间电感值
    center_value = normalized_data[SENSOR_HC];
    
    // 计算信号强度指标 - 所有电感平均值
    signal_strength = (sum_outer + sum_middle + sum_vertical + center_value) / 7.0f;
    signal_strength_value = signal_strength; // 保存信号强度指标




    // 计算差比和，使用平滑过渡函数代替硬阈值，避免在临界值附近产生跳变
    // 外侧电感平滑过渡
    if(sum_outer > 16.0f)
        ratio_outer = diff_outer / sum_outer;
    else if(sum_outer < 3.0f)
        ratio_outer = 0;
    else
        ratio_outer = (diff_outer / sum_outer) * (sum_outer - 3.0f) / 7.0f; // 5-12范围内线性过渡
        
    // 中间电感平滑过渡
    if(sum_middle > 16.0f)
        ratio_middle = diff_middle / sum_middle;
    else if(sum_middle < 3.0f)
        ratio_middle = 0;
    else
        ratio_middle = (diff_middle / sum_middle) * (sum_middle - 3.0f) / 7.0f; // 5-12范围内线性过渡
    
    // 纵向电感平滑过渡
    if(sum_vertical > 16.0f)
        ratio_vertical = diff_vertical / sum_vertical;
    else if(sum_vertical < 3.0f)
        ratio_vertical = 0;
    else
        ratio_vertical = (diff_vertical / sum_vertical) * (sum_vertical - 3.0f) / 7.0f; // 5-12范围内线性过渡
    
	
	
	
    // 赛道类型识别 - 根据七电感特征判断
    if (track_type == WEIGHT_STRAIGHT || track_type == 4) // 0. 当前认为是普通赛道时，尝试判断特殊赛道
    {    
        // 1. 直角弯道特征
        if(((normalized_data[SENSOR_VL] > 65.0f && normalized_data[SENSOR_HR] < 35.0f && normalized_data[SENSOR_VR] < 20.0f) || //左转
                (normalized_data[SENSOR_VR] > 65.0f && normalized_data[SENSOR_HL] < 30.0f && normalized_data[SENSOR_VL] < 20.0f)) &&  //右转
								normalized_data[SENSOR_HC] < 75.0f && 
                signal_strength > 25.0f && signal_strength < 50.0f) // 调整信号强度范围
        {
            track_type = WEIGHT_RIGHT_ANGLE; // 直角弯道
        }
				else if (((normalized_data[SENSOR_HC] > 60.0f && normalized_data[SENSOR_HMR] > 88.0f && normalized_data[SENSOR_VL] > 50.0f && normalized_data[SENSOR_VR] > 75.0f) ||  //逆时针
                (normalized_data[SENSOR_HC] > 80.0f && normalized_data[SENSOR_HML] > 80.0f && normalized_data[SENSOR_HMR] < 45.0f && normalized_data[SENSOR_VL] > 75.0f && normalized_data[SENSOR_VR] > 45.0f)) && 
                track_ten_flag == 1 && signal_strength > 50.0f ) 
				{
					track_type = 2; //十字圆环
					track_ten_flag = 0; 
					ten_change_flag = 1;//感应到入环，延时2s再让track_ten_flag=1
					
				}
        else if((normalized_data[SENSOR_HR] > 70.0f && normalized_data[SENSOR_HC] > 95.0f && ((normalized_data[SENSOR_HR] + normalized_data[SENSOR_VR]) - (normalized_data[SENSOR_HL] + normalized_data[SENSOR_VL]) > 90.0f))  //右环岛
                 && signal_strength > 50.0f )    
        {
            track_type = 3;// 环岛
        }
    }
    else if (track_type == WEIGHT_RIGHT_ANGLE) // 1. 直角弯道
	{
		if (normalized_data[SENSOR_VL] > 60.0f && normalized_data[SENSOR_VR] < 20.0f )
		{
			track_type_zj = 1; //左转
			
		}
		else if (normalized_data[SENSOR_VR] > 70.0f && normalized_data[SENSOR_VL] < 20.0f )
		{
			track_type_zj = 2; //右转
		}
		
		if (track_type_zj != 0)
		{
			// 回到直道 - 可选:增加 signal_strength < 45.0f 判断
			if (normalized_data[SENSOR_VR] < 20.0f && normalized_data[SENSOR_VL] < 20.0f ) 
			{
				track_type = WEIGHT_STRAIGHT; 
				track_type_zj = 0;
			}
//			if (signal_strength > 50) // 直角右拐进圆环的特殊点
//			{
//				track_type = WEIGHT_ROUNDABOUT; 
//				// track_type_zj = 0;
//			    // weight_outer = 0.4;  // 换成直道的权
//			    // weight_middle = 0.1;
//			    // weight_vertical = 0.1;
//			    // filter_param = track_weights[WEIGHT_STRAIGHT].filter_param;
//			    // max_change_rate = track_weights[WEIGHT_STRAIGHT].max_change_rate;
//			}
		}
	}
    else if (track_type == WEIGHT_CROSS) // 2. 十字圆环
    {
		// 出环  
		if (((normalized_data[SENSOR_HC] > 70.0f && normalized_data[SENSOR_HML] > 75.0f && normalized_data[SENSOR_VL] > 80.0f && normalized_data[SENSOR_VR] > 70.0f)  || //逆时针
					 (normalized_data[SENSOR_HC] > 80.0f && normalized_data[SENSOR_HMR] > 80.0f && normalized_data[SENSOR_VL] > 70.0f && normalized_data[SENSOR_VR] > 80.0f )) &&
						track_ten_flag == 1 && signal_strength > 50.0f )  //顺时针
		 {
			track_type = WEIGHT_STRAIGHT; //回直道
			track_ten_flag = 0;
			ten_change_flag = 1; //感应到出环延时2s再让track_ten_flag=1	
		 }
	}
    else if (track_type == WEIGHT_ROUNDABOUT) // 3. 环岛   
    {
        if(normalized_data[SENSOR_HR] > 70.0f && normalized_data[SENSOR_HL] < 35.0f && track_route == 0)
        {
            // 右环岛
            track_route = 1;
		    track_route_status = 1;
        }
        else if(normalized_data[SENSOR_HR] < 30.0f && normalized_data[SENSOR_HL] > 70.0f && track_route == 0)
        {
            // 左环岛
            track_route = 2;
			track_route_status = 1;
        }
		if(track_route_status == 2 &&(normalized_data[SENSOR_HC] < 70.0f && normalized_data[SENSOR_HML] < 30.0f && normalized_data[SENSOR_HMR] > 80.0f && normalized_data[SENSOR_VR] > 75.0f &&normalized_data[SENSOR_HL] > 20.0f && signal_strength > 40.0f)) //右环
		{
//			track_route = 0;
			track_route_status = 3;
//			track_type == WEIGHT_RIGHT_ANGLE; // 检验位点
		}
    }

    // 4. 超出置0
//   if(normalized_data[SENSOR_HC] < 2.0f && normalized_data[SENSOR_HMR] < 2.0f && normalized_data[SENSOR_HML] < 2.0f)
//   {
//       track_type = WEIGHT_STRAIGHT;
//       track_route = 0;
//       track_route_status = 0;
//	   track_type_zj = 0;
//   }
    
    // 根据赛道类型和信号强度调整权重
   switch(track_type)
   {
       case WEIGHT_STRAIGHT: // 普通赛道
           // 使用直道权重
           weight_outer = track_weights[WEIGHT_STRAIGHT].weight_outer;
           weight_middle = track_weights[WEIGHT_STRAIGHT].weight_middle;
           weight_vertical = track_weights[WEIGHT_STRAIGHT].weight_vertical;
           filter_param = track_weights[WEIGHT_STRAIGHT].filter_param;
           max_change_rate = track_weights[WEIGHT_STRAIGHT].max_change_rate;
           break;
           
       case WEIGHT_RIGHT_ANGLE: // 直角弯道
           // 使用直角弯道权重
           weight_outer = track_weights[WEIGHT_RIGHT_ANGLE].weight_outer;
           weight_middle = track_weights[WEIGHT_RIGHT_ANGLE].weight_middle;
           weight_vertical = track_weights[WEIGHT_RIGHT_ANGLE].weight_vertical;
           filter_param = track_weights[WEIGHT_RIGHT_ANGLE].filter_param;
           max_change_rate = track_weights[WEIGHT_RIGHT_ANGLE].max_change_rate;
           break;
           
       case WEIGHT_CROSS: // 十字圆环
           // 使用十字圆环权重
           weight_outer = track_weights[WEIGHT_CROSS].weight_outer;
           weight_middle = track_weights[WEIGHT_CROSS].weight_middle;
           weight_vertical = track_weights[WEIGHT_CROSS].weight_vertical;
           filter_param = track_weights[WEIGHT_CROSS].filter_param;
           max_change_rate = track_weights[WEIGHT_CROSS].max_change_rate;
           break;
           
       case WEIGHT_ROUNDABOUT: // 环岛
           // 使用环岛权重
           weight_outer = track_weights[WEIGHT_ROUNDABOUT].weight_outer;
           weight_middle = track_weights[WEIGHT_ROUNDABOUT].weight_middle;
           weight_vertical = track_weights[WEIGHT_ROUNDABOUT].weight_vertical;
           filter_param = track_weights[WEIGHT_ROUNDABOUT].filter_param;
           max_change_rate = track_weights[WEIGHT_ROUNDABOUT].max_change_rate;
           break;
       default:
           // 使用默认的直道权重
           weight_outer = track_weights[WEIGHT_STRAIGHT].weight_outer;
           weight_middle = track_weights[WEIGHT_STRAIGHT].weight_middle;
           weight_vertical = track_weights[WEIGHT_STRAIGHT].weight_vertical;
           filter_param = track_weights[WEIGHT_STRAIGHT].filter_param;
           max_change_rate = track_weights[WEIGHT_STRAIGHT].max_change_rate;
           break;
   }
    
    // 特殊情况处理：当所有电感值都很小时，可能已经偏离赛道
    if(sum_outer < 10.0f && sum_middle < 10.0f && sum_vertical < 10.0f && center_value < 10.0f)
    {
//        if(last_pos > 0)
//            return (last_pos + 10);  // 向右偏离
//        else
//            return (last_pos - 10); // 向左偏离
		
		return last_pos;
    }
    
    // 当中心电感大于阈值时，认为车辆接近中心，对位置进行修正
    if(center_value > 60.0f) {
        // 修正系数，当中心电感强度高时，修正系数大
        center_correction = (center_value - 40.0f) / 60.0f * 0.5f;  // 最大修正50%
    }
    
    // 三组差比和加权平均计算位置
    pos = (int16)((ratio_outer * weight_outer + 
                   ratio_middle * weight_middle + 
                   ratio_vertical * weight_vertical) * 100.0f);
    
    // 应用中心电感修正 - 向中心线拉近
    pos = (int16)(pos * (1.0f - center_correction));
    
    // 限制范围在-100到100之间
    if(pos > 100) pos = 100;
    if(pos < -100) pos = -100;
    
    // 位置变化量限制，防止突变
    position_change = pos - last_pos;
    if(position_change > max_change_rate)
        pos = last_pos + max_change_rate;
    else if(position_change < -max_change_rate)
        pos = last_pos - max_change_rate;
    
    // 应用低通滤波，平滑位置变化
    pos = (int16)(filter_param * pos + (1-filter_param) * last_pos);
    
    // 如果信号强度高，增强滤波效果
    if(signal_strength > 60.0f) {
        // 应用三点平均滤波，进一步平滑
        pos = (pos + last_pos + very_last_pos) / 3;
    }
		
		
//		if (track_type == 0 && (pos <= 10 || pos >= -10))
//		{
//				speed_count++;
//			
//				if (speed_count >= 12)
//				{
//						track_type = 4;
//						speed_count = 0;
//				}
//		}
//		else
//		{
//				speed_count = 0;
//		}
//		
//		if (track_type == 4 && (pos > 10 || pos < -10))
//		{
//			track_type = 0;
//		}
		
    
    // 更新历史位置值
    very_very_last_pos = very_last_pos;
    very_last_pos = last_pos;
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
    uint16 threshold = 175;       // 阈值，需要根据七电感的实际情况调整（增加）
    static uint8 out_of_track_count = 0;    // 连续检测到脱离赛道的次数计数器
    static uint8 in_track_count = 0;        // 连续检测到在轨道上的次数计数器
    static uint8 protection_triggered = 0;  // 保护触发标志位，1表示已触发保护
    uint8 i;
    uint8 trigger_reason = 0;     // 记录触发原因，用于调试
    
    if (startKeyFlag == 1)
	{
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
			trigger_reason = 1;
		}
		
		// 2. 归一化后的值都很小，说明可能脱离赛道
		if(normalized_data[SENSOR_HL] < 5.0f && normalized_data[SENSOR_VL] < 5.0f && 
		   normalized_data[SENSOR_HML] < 5.0f && normalized_data[SENSOR_HC] < 5.0f && 
		   normalized_data[SENSOR_HMR] < 5.0f && normalized_data[SENSOR_VR] < 5.0f && 
		   normalized_data[SENSOR_HR] < 5.0f)
		{
			is_out_of_track = 1;
			trigger_reason = 2;
		}
		
		// 3. 位置偏差过大，说明可能偏离赛道太多
		if(position < -90 || position > 90)
		{
			// 只有当电感值总和也较小时才判断为出赛道
			if(sum_value < threshold * 2)
			{
				is_out_of_track = 1;
				trigger_reason = 3;
			}
		}
		
		// 连续检测逻辑，防止偶然的低值导致误判
		if(is_out_of_track)
		{
			out_of_track_count++;
			in_track_count = 0;  // 重置在轨道上的计数
			
			if(out_of_track_count >= 30 && !protection_triggered)  // 连续5次检测到脱离赛道才触发保护
			{
				protection_triggered = 1;
				// 这里可以输出触发保护的信息，用于调试
	////            sprintf(g_TxData, "Protection triggered! Reason: %d, Sum: %d\n", trigger_reason, sum_value);
	////            uart_putstr(UART_4, g_TxData);
			}
		}
		// else
		// {
		//     // 如果检测正常，计数器增加
		//     in_track_count++;
		//     if(out_of_track_count > 0)
		//         out_of_track_count--;
				
		//     // 自动恢复机制：连续20次检测到正常，则解除保护状态
		//     if(in_track_count >= 20 && protection_triggered)
		//     {
		//         protection_triggered = 0;
		//         out_of_track_count = 0;
		//         in_track_count = 0;
		//         // 可以输出自动恢复的信息，用于调试
		//         // sprintf(g_TxData, "Protection auto reset!\n");
		//         // uart_putstr(UART_4, g_TxData);
		//     }
		// }
		
		return protection_triggered;
	}
	return 0;
}



// 显示电磁传感器数据
void display_electromagnetic_data(void)
{
    // 显示原始滤波数据和归一化数据
    ips114_showstr_simspi(0,0,"HL:");   
    ips114_showuint16_simspi(3*8, 0, result[SENSOR_HL]);
    ips114_showstr_simspi(9*8,0,"N:");
    ips114_showfloat_simspi(11*8, 0, normalized_data[SENSOR_HL], 2, 2);
    
    ips114_showstr_simspi(0,1,"VL:");  
    ips114_showuint16_simspi(3*8, 1, result[SENSOR_VL]);
    ips114_showstr_simspi(9*8,1,"N:");
    ips114_showfloat_simspi(11*8, 1, normalized_data[SENSOR_VL], 2, 2);
    
    ips114_showstr_simspi(0,2,"HML:");  
    ips114_showuint16_simspi(4*8, 2, result[SENSOR_HML]);
    ips114_showstr_simspi(9*8,2,"N:");
    ips114_showfloat_simspi(11*8, 2, normalized_data[SENSOR_HML], 2, 2);
    
    ips114_showstr_simspi(0,3,"HC:");   
    ips114_showuint16_simspi(3*8, 3, result[SENSOR_HC]);
    ips114_showstr_simspi(9*8,3,"N:");
    ips114_showfloat_simspi(11*8, 3, normalized_data[SENSOR_HC], 2, 2);
    
    ips114_showstr_simspi(0,4,"HMR:");   
    ips114_showuint16_simspi(4*8, 4, result[SENSOR_HMR]);
    ips114_showstr_simspi(9*8,4,"N:");
    ips114_showfloat_simspi(11*8, 4, normalized_data[SENSOR_HMR], 2, 2);
    
    ips114_showstr_simspi(0,5,"VR:");   
    ips114_showuint16_simspi(3*8, 5, result[SENSOR_VR]);
    ips114_showstr_simspi(9*8,5,"N:");
    ips114_showfloat_simspi(11*8, 5, normalized_data[SENSOR_VR], 2, 2);
    
    ips114_showstr_simspi(0,6,"HR:");   
    ips114_showuint16_simspi(3*8, 6, result[SENSOR_HR]);
    ips114_showstr_simspi(9*8,6,"N:");
    ips114_showfloat_simspi(11*8, 6, normalized_data[SENSOR_HR], 2, 2);
    
    // 显示位置和差比和数据
    ips114_showstr_simspi(0,7,"Pos:");
    ips114_showint16_simspi(5*8, 7, position);
    
    // 显示保护状态
    ips114_showstr_simspi(10*8,7,"P:");
    ips114_showuint8_simspi(12*8, 7, protection_flag);
}

/**
 * @brief DMA数据搬运测试函数
 * @return 测试结果，0表示成功，非0表示错误码
 */
uint8 test_dma_data_transfer(void)
{
    uint8 i, j;
    uint8 error_code = DMA_ADC_ERROR_NONE;
    uint16 min_value = 65535, max_value = 0;
	uint16 timeout_count = 0;
    uint8 test = 0;
    
    char test_buffer[50];
    
    // 打印初始状态
    uart_putstr(UART_4, "DMA Test Starting \r\n");
    uart_putstr(UART_4, "DMA Flag before test ");
    if(g_adc_dma_completed_flag) 
        uart_putstr(UART_4, "1\r\n");
    else
        uart_putstr(UART_4, "0\r\n");
    
    // 清除DMA完成标志
    g_adc_dma_completed_flag = 0;
    
    // 触发DMA ADC转换
    uart_putstr(UART_4, "Triggering DMA conversion...\r\n");
    start_adc_dma_conversion();
    
    // 等待DMA完成，最多等待100ms
    uart_putstr(UART_4, "Waiting for DMA completion...\r\n");

    while (!g_adc_dma_completed_flag && timeout_count < 1000)
    {
        delay_ms(1);
        timeout_count++;
        
        // 每200ms打印一次状态
        if(timeout_count % 200 == 0) {
            uart_putstr(UART_4, "Still waiting... count");
            sprintf(test_buffer, "%d\r\n", timeout_count);
            uart_putstr(UART_4, test_buffer);
        }
    }
    
    // 检查是否超时
    if (timeout_count >= 1000)
    {
        uart_putstr(UART_4, "TIMEOUT ERROR DMA completion flag not set!\r\n");
        return DMA_ADC_ERROR_TIMEOUT;
    }
    
    // 打印DMA采集的原始数据
    uart_putstr(UART_4, "DMA Test Raw data transfer test\r\n");
    
    // // 测试数据缓冲区地址
    // sprintf(test_buffer, "DMA_ADC_RXAH %08LX\r\n ", DMA_ADC_RXAH);
    // uart_putstr(UART_4, test_buffer);

    // sprintf(test_buffer, "DMA_ADC_RXAL %08LX\r\n ", DMA_ADC_RXAL);
    // uart_putstr(UART_4, test_buffer);

    // sprintf(test_buffer, "(uint16)AdcDmaBuffer %p\r\n ", (uint16)AdcDmaBuffer);
    // uart_putstr(UART_4, test_buffer);

    // 检查每个通道的数据
    for (i = 0; i < ADC_DMA_USED_CHANNEL_COUNT; i++)
    {
        sprintf(test_buffer, "Channel %d\r\n ", i);
        uart_putstr(UART_4, test_buffer);
        

        // 打印该通道的所有采样值
        for (j = 0; j < ADC_DMA_SAMPLES_PER_CHANNEL_NUM; j++)
        {
            sprintf(test_buffer, "%d\r\n ", AdcDmaBuffer[i][j]);
            uart_putstr(UART_4, test_buffer);
            
            // 统计最大最小值，用于检查数据合理性
            if (AdcDmaBuffer[i][j] < min_value)
                min_value = AdcDmaBuffer[i][j];
            if (AdcDmaBuffer[i][j] > max_value)
                max_value = AdcDmaBuffer[i][j];
        }
        
        sprintf(test_buffer, "\r\n");
        uart_putstr(UART_4, test_buffer);
    }
    

    test = 0;

    sprintf(test_buffer, "AdcDmaBuffer[test][8] %d\r\n ", AdcDmaBuffer[test][8]);
    uart_putstr(UART_4, test_buffer);
    sprintf(test_buffer, "AdcDmaBuffer[test][0] %d\r\n ", AdcDmaBuffer[test][0]);
    uart_putstr(UART_4, test_buffer);
    sprintf(test_buffer, "AdcDmaBuffer[test][1] %d\r\n ", AdcDmaBuffer[test][1]);
    uart_putstr(UART_4, test_buffer);
    sprintf(test_buffer, "AdcDmaBuffer[test][10] %d\r\n ", AdcDmaBuffer[test][10]);
    uart_putstr(UART_4, test_buffer);
    sprintf(test_buffer, "AdcDmaBuffer[test][11] %d\r\n ", AdcDmaBuffer[test][11]);
    uart_putstr(UART_4, test_buffer);

    // sprintf(test_buffer, "AdcDmaBuffer[1][0] %d\r\n ", AdcDmaBuffer[1][0]);
    // uart_putstr(UART_4, test_buffer);
    // sprintf(test_buffer, "AdcDmaBuffer[2][0] %d\r\n ", AdcDmaBuffer[2][0]);
    // uart_putstr(UART_4, test_buffer);
    // sprintf(test_buffer, "AdcDmaBuffer[3][0] %d\r\n ", AdcDmaBuffer[3][0]);
    // uart_putstr(UART_4, test_buffer);
    // sprintf(test_buffer, "AdcDmaBuffer[4][0] %d\r\n ", AdcDmaBuffer[4][0]);
    // uart_putstr(UART_4, test_buffer);
    // sprintf(test_buffer, "AdcDmaBuffer[5][0] %d\r\n ", AdcDmaBuffer[5][0]);
    // uart_putstr(UART_4, test_buffer);
    // sprintf(test_buffer, "AdcDmaBuffer[6][0] %d\r\n ", AdcDmaBuffer[6][0]);
    // uart_putstr(UART_4, test_buffer);



    // 检查数据是否在合理范围内
    // ADC值通常在0-4095范围内（12位ADC）
    sprintf(test_buffer, "Min value  %d, Max value  %d\r\n", min_value, max_value);
    uart_putstr(UART_4, test_buffer);
    
    if (min_value == 0 && max_value == 0)
    {
        uart_putstr(UART_4, "ERROR  All values are zero, possible DMA failure\r\n");
        error_code = DMA_ADC_ERROR_INVALID_DATA;
    }
    else if (max_value > 4095)  // 假设使用12位ADC
    {
        uart_putstr(UART_4, "ERROR  Values out of range, possible ADC config error\r\n");
        error_code = DMA_ADC_ERROR_INVALID_DATA;
    }
    else
    {
        uart_putstr(UART_4, "SUCCESS  Data transfer successful\r\n");
    }
    
    // 再次启动DMA转换以保持连续性
    start_adc_dma_conversion();
    
    return error_code;
}

/**
 * @brief 运行所有DMA ADC测试
 */
void run_electromagnetic_dma_tests(void)
{
    char test_buffer[50];
    uint8 result1 = 0;
    
    uart_putstr(UART_4, "Starting DMA ADC tests...\r\n");
    
    // 确保DMA已初始化
    uart_putstr(UART_4, "Checking DMA initialization status...\r\n");
    
    // 检查DMA状态寄存器
    sprintf(test_buffer, "DMA_ADC_STA 0x%02X\r\n", DMA_ADC_STA);
    uart_putstr(UART_4, test_buffer);
    
    // 检查DMA控制寄存器
    sprintf(test_buffer, "DMA_ADC_CR 0x%02X\r\n", DMA_ADC_CR);
    uart_putstr(UART_4, test_buffer);
    
    // 检查DMA中断使能状态
    sprintf(test_buffer, "DMA_ADC_IE %d\r\n", (DMA_ADC_CR & 0x80) ? 1 : 0);
    uart_putstr(UART_4, test_buffer);
    
    // 测试1: DMA数据搬运测试
    uart_putstr(UART_4, "Running Test 1 DMA data transfer test\r\n");
    result1 = test_dma_data_transfer();
    sprintf(test_buffer, "Test 1 result %d\r\n", result1);
    uart_putstr(UART_4, test_buffer);
    
    // 延时一段时间，确保DMA完成
    delay_ms(100);
    
    // 再次检查DMA状态
    sprintf(test_buffer, "Final DMA_ADC_STA 0x%02X\r\n", DMA_ADC_STA);
    uart_putstr(UART_4, test_buffer);
    
    // 后续可添加其他测试
    // uint8 result2 = test_channel_mapping();
    // uint8 result3 = test_dma_interrupt();
    
    uart_putstr(UART_4, "DMA ADC tests completed\r\n");
}

