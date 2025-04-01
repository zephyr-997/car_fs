#include "../inc/filter.h"


const float imu693kf_Q = 0.17;
const float imu693kf_R = 0.20;


KalmanFilter imu693_kf;


void Kalman_Init(KalmanFilter* kf, float F, float B, float Q, float R, float initial_x)
{
    kf->F = F;
    kf->B = B;
    kf->Q = Q;
    kf->R = R;
    kf->P = 1.0;       // 初始协方差
    kf->x = initial_x;
}

void Kalman_Predict(KalmanFilter* kf, float u)
{
    kf->x = kf->F * kf->x + kf->B * u;
    kf->P = kf->F * kf->P * kf->F + kf->Q;
}

float Kalman_Update(KalmanFilter* kf, float z)
{
    kf->K = kf->P / (kf->P + kf->R);
    kf->x += kf->K * (z - kf->x);
    kf->P *= (1 - kf->K);
    return kf->x;
}