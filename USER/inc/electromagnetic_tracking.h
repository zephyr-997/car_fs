#ifndef __ELECTROMAGNETIC_TRACKING_H__
#define __ELECTROMAGNETIC_TRACKING_H__

#include "headfile.h"
#include "STC32G_ADC.h"
#include "STC32G_DMA.h"

// 电感通道定义
#define ADC_HL  ADC_P06  // 左侧横向电感
#define ADC_VL  ADC_P05  // 左侧纵向电感
#define ADC_HML ADC_P01  // 左中横向电感
#define ADC_HC  ADC_P11  // 中间横向电感
#define ADC_HMR ADC_P13  // 右中横向电感
#define ADC_VR  ADC_P14  // 右侧纵向电感
#define ADC_HR  ADC_P00  // 右侧横向电感

// ADC DMA通道映射
#define DMA_ADC_CH_HL  14  // ADC_P06 -> CH14
#define DMA_ADC_CH_VL  13  // ADC_P05 -> CH13
#define DMA_ADC_CH_HML 9   // ADC_P01 -> CH9
#define DMA_ADC_CH_HC  1   // ADC_P11 -> CH1
#define DMA_ADC_CH_HMR 3   // ADC_P13 -> CH3
#define DMA_ADC_CH_VR  4   // ADC_P14 -> CH4
#define DMA_ADC_CH_HR  8   // ADC_P00 -> CH8

// DMA ADC配置
#define ADC_DMA_SAMPLES_PER_CHANNEL ADC_4_Times  // 每个通道DMA采集4次
#define ADC_DMA_SAMPLES_PER_CHANNEL_NUM  12        // 对应的数值
#define ADC_DMA_USED_CHANNEL_COUNT 7             // 使用的通道数

// 组合所有ADC通道的位掩码
#define ELECTROMAGNETIC_DMA_CHANNELS ( (1 << DMA_ADC_CH_HL)  | \
                                     (1 << DMA_ADC_CH_VL)  | \
                                     (1 << DMA_ADC_CH_HML) | \
                                     (1 << DMA_ADC_CH_HC)  | \
                                     (1 << DMA_ADC_CH_HMR) | \
                                     (1 << DMA_ADC_CH_VR)  | \
                                     (1 << DMA_ADC_CH_HR)    )

// 电感数组定义
#define SENSOR_COUNT 7   // 电感个数
#define HISTORY_COUNT 5  // 滤波次数，当前只存储最新值

// 赛道类型索引，与track_type对应
#define WEIGHT_STRAIGHT    0  // 直道
#define WEIGHT_RIGHT_ANGLE 1  // 直角弯道
#define WEIGHT_CROSS       2  // 十字圆环
#define WEIGHT_ROUNDABOUT  3  // 环岛

// DMA ADC错误码定义
#define DMA_ADC_ERROR_NONE          0
#define DMA_ADC_ERROR_TIMEOUT       1  // DMA转换超时
#define DMA_ADC_ERROR_INVALID_DATA  2  // 数据异常
#define DMA_ADC_ERROR_CONFLICT      3  // 资源冲突

// 测试模式开关
#define DMA_ADC_TEST_MODE  1

// 定义电感权重结构体
typedef struct {
    float weight_outer;    // 外侧电感权重(HL和HR)
    float weight_middle;   // 中间电感权重(HML和HMR)
    float weight_center;   // 中心电感权重(HC)
    float weight_vertical; // 纵向电感权重(VL和VR)
    float filter_param;    // 滤波系数，可调
    int16 max_change_rate; // 允许的最大变化率
    char *name;            // 赛道类型名称，便于调试
} TrackWeights;


// 电感类型枚举
typedef enum {
    SENSOR_HL  = 0,  // 左侧横向电感
    SENSOR_VL  = 1,  // 左侧纵向电感-8
    SENSOR_HML = 2,  // 左中横向电感
    SENSOR_HC  = 3,  // 中间横向电感
    SENSOR_HMR = 4,  // 右中横向电感
    SENSOR_VR  = 5,  // 右侧纵向电感
    SENSOR_HR  = 6   // 右侧横向电感
} sensor_type_e;

// 函数声明
// void electromagnetic_init(void);               // 初始化电磁传感器 - 已被electromagnetic_dma_init替代
void average_filter(void);                     // 递推均值滤波函数
void mid_filter(void);                         // 中位值滤波函数
void update_min_max_values(void);              // 更新每个电感的最大最小值
void normalize_sensors(void);                  // 归一化电感数据
int16 calculate_position_improved(void);       // 改进版计算位置
uint8 check_electromagnetic_protection(void);  // 电磁保护逻辑函数

// DMA ADC相关函数声明
void electromagnetic_dma_init(void);           // 初始化电磁传感器DMA ADC
void process_adc_dma_data(void);               // 处理DMA ADC数据
void start_adc_dma_conversion(void);           // 启动DMA ADC转换

// DMA ADC测试函数声明
#ifdef DMA_ADC_TEST_MODE
uint8 test_dma_data_transfer(void);            // DMA数据搬运测试
uint8 test_channel_mapping(void);              // 通道映射测试
uint8 test_dma_interrupt(void);                // DMA中断响应测试

/**
 * @brief 运行所有DMA ADC测试
 */
void run_electromagnetic_dma_tests(void);
#endif

// 外部变量声明
extern uint16 adc_fliter_data[SENSOR_COUNT][HISTORY_COUNT]; // 滤波后的值
extern float result[SENSOR_COUNT];                // 电感结果数据
extern float normalized_data[SENSOR_COUNT];       // 归一化后的电感数据，数组形式
extern uint16 min_value[SENSOR_COUNT];            // 每个电感的最小值
extern uint16 max_value[SENSOR_COUNT];            // 每个电感的最大值
extern int16 position;                         // 位置偏差
extern uint8 protection_flag;                  // 保护标志
extern float signal_strength_value;            // 信号强度指标

// DMA ADC相关变量声明
extern uint8_t xdata AdcDmaBuffer[ADC_DMA_USED_CHANNEL_COUNT][ADC_DMA_SAMPLES_PER_CHANNEL_NUM];  // DMA ADC缓冲区
extern volatile uint8 g_adc_dma_completed_flag;  // DMA ADC数据就绪标志（统一标志）

//电磁位置计算变量
extern float filter_param;   // 滤波系数，可调
extern uint8 track_type;        // 赛道类型：0-普通，1-十字，2-环岛，3-直角弯道
extern uint8 track_type_last;         // 赛道类型：0-普通，1-直角弯道，2-十字圆环，3-环岛

extern uint8 track_route;      // 赛道路径：1-左转，2-右转
extern uint8 track_route_status;
extern uint8 track_type_zj;
extern uint8 track_ten_flag;
// 调试相关函数
void display_electromagnetic_data(void);       // 显示电磁传感器数据

// 电磁保护逻辑变量
extern uint8 protection_flag;

#endif 

