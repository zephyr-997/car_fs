#ifndef __ELECTROMAGNETIC_TRACKING_H__
#define __ELECTROMAGNETIC_TRACKING_H__

#include "headfile.h"

// 电感通道定义
#define ADC_L   ADC_P00  // 左水平电感
#define ADC_LM  ADC_P01  // 左垂直电感
#define ADC_RM  ADC_P05  // 右垂直电感
#define ADC_R   ADC_P06  // 右水平电感

// 电感数组定义
#define SENSOR_COUNT 4   // 电感个数
#define HISTORY_COUNT 5  // 滤波次数，当前只存储最新值

// 电感类型枚举
typedef enum {
    SENSOR_L = 0,
    SENSOR_LM = 1,
    SENSOR_RM = 2,
    SENSOR_R = 3
} sensor_type_e;

// 函数声明
void electromagnetic_init(void);               // 初始化电磁传感器
uint16 get_adc(uint16 i);                      // 获取ADC的值
void average_filter(void);                     // 递推均值滤波函数
void mid_filter(void);                         // 中位值滤波函数
void update_min_max_values(void);              // 更新每个电感的最大最小值
void normalize_sensors(void);                  // 归一化电感数据
int16 calculate_position(void);                // 计算位置（差比和加权算法）
uint8 check_electromagnetic_protection(void);  // 电磁保护逻辑函数

// 外部变量声明
extern uint16 adc_fliter_data[SENSOR_COUNT][HISTORY_COUNT]; // 滤波后的值
extern float result[SENSOR_COUNT];                // 电感结果数据
extern float normalized_data[SENSOR_COUNT];       // 归一化后的电感数据，数组形式
extern uint16 min_value[SENSOR_COUNT];            // 每个电感的最小值
extern uint16 max_value[SENSOR_COUNT];            // 每个电感的最大值
extern int16 position;                         // 位置偏差
extern uint8 protection_flag;                  // 保护标志

// 调试相关函数
void display_electromagnetic_data(void);       // 显示电磁传感器数据

#endif 

