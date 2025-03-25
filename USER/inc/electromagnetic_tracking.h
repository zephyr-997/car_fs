#ifndef __ELECTROMAGNETIC_TRACKING_H__
#define __ELECTROMAGNETIC_TRACKING_H__

#include "headfile.h"

// 电感通道定义
#define ADC_L   ADC_P00  // 左水平电感
#define ADC_LM  ADC_P01  // 左垂直电感
#define ADC_RM  ADC_P05  // 右垂直电感
#define ADC_R   ADC_P06  // 右水平电感

// 函数声明
void electromagnetic_init(void);                      // 初始化电磁传感器
uint16 adc_mean_filter(ADCN_enum adcn, uint8 count);  // 均值滤波函数
void update_min_max_values(void);                     // 更新每个电感的最大最小值
void normalize_sensors(void);                         // 归一化电感数据
int16 calculate_position(void);                       // 计算位置（差比和加权算法）
uint8 check_electromagnetic_protection(void);         // 电磁保护逻辑函数

// 外部变量声明
extern uint16 filtered_L, filtered_LM, filtered_RM, filtered_R;  // 滤波后数据
extern float normalized_L, normalized_LM, normalized_RM, normalized_R;  // 归一化数据
extern int16 position;  // 位置偏差
extern uint8 protection_flag;  // 保护标志

// 调试相关函数
void display_electromagnetic_data(void);  // 显示电磁传感器数据

#endif 

