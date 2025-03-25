#ifndef __FILTER_H
#define __FILTER_H

#include "headfile.h"


typedef struct {
    float F;        // 状态转移系数（系统动力学）
    float B;        // 控制输入系数
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float P;        // 估计误差协方差
    float K;        // 卡尔曼增益
    float x;        // 状态估计值（gyroz）
} KalmanFilter;


extern const float imu693kf_Q;
extern const float imu693kf_R;


extern KalmanFilter imu693_kf;


void Kalman_Init(KalmanFilter* kf, float F, float B, float Q, float R, float initial_x);
void Kalman_Predict(KalmanFilter* kf, float u);
float Kalman_Update(KalmanFilter* kf, float z);


#endif
