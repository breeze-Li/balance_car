#include "Kalman.h"

// 初始化滤波器
void Kalman_Init(KalmanSpeedFilter *kf, float dt) {
    // 初始状态：速度为0，加速度为0
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    
    // 初始协方差：对速度确定性较低，加速度更不确定
    kf->P[0][0] = 100.0f;  // 速度方差
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1000.0f; // 加速度方差较大
    
    // 状态转移矩阵：x_k = A * x_{k-1}
    // 速度 = 上一时刻速度 + 加速度 * dt
    // 加速度 = 上一时刻加速度 (假设恒定)
    kf->A[0][0] = 1.0f;
    kf->A[0][1] = dt;
    kf->A[1][0] = 0.0f;
    kf->A[1][1] = 1.0f;
    
    // 观测矩阵：测量速度
    kf->H[0][0] = 1.0f;
    kf->H[0][1] = 0.0f;
    
    // 过程噪声（需要根据实际调参）
    kf->Q[0][0] = 0.1f;   // 速度过程噪声
    kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;
    kf->Q[1][1] = 1.0f;   // 加速度过程噪声
    
    // 测量噪声（T法原始速度的噪声方差）
    kf->R = 10.0f;
    
    kf->dt = dt;
}

// 卡尔曼预测步
void Kalman_Predict(KalmanSpeedFilter *kf) {
    // 临时变量存储新的P
    float new_P[2][2];
    
    // 1. 预测状态: x = A * x
    float new_x0 = kf->A[0][0] * kf->x[0] + kf->A[0][1] * kf->x[1];
    float new_x1 = kf->A[1][0] * kf->x[0] + kf->A[1][1] * kf->x[1];
    kf->x[0] = new_x0;
    kf->x[1] = new_x1;
    
    // 2. 预测协方差: P = A * P * A^T + Q
    // 第一步：temp = A * P
    float temp[2][2];
    temp[0][0] = kf->A[0][0] * kf->P[0][0] + kf->A[0][1] * kf->P[1][0];
    temp[0][1] = kf->A[0][0] * kf->P[0][1] + kf->A[0][1] * kf->P[1][1];
    temp[1][0] = kf->A[1][0] * kf->P[0][0] + kf->A[1][1] * kf->P[1][0];
    temp[1][1] = kf->A[1][0] * kf->P[0][1] + kf->A[1][1] * kf->P[1][1];
    
    // 第二步：new_P = temp * A^T + Q
    new_P[0][0] = temp[0][0] * kf->A[0][0] + temp[0][1] * kf->A[0][1] + kf->Q[0][0];
    new_P[0][1] = temp[0][0] * kf->A[1][0] + temp[0][1] * kf->A[1][1] + kf->Q[0][1];
    new_P[1][0] = temp[1][0] * kf->A[0][0] + temp[1][1] * kf->A[0][1] + kf->Q[1][0];
    new_P[1][1] = temp[1][0] * kf->A[1][0] + temp[1][1] * kf->A[1][1] + kf->Q[1][1];
    
    kf->P[0][0] = new_P[0][0];
    kf->P[0][1] = new_P[0][1];
    kf->P[1][0] = new_P[1][0];
    kf->P[1][1] = new_P[1][1];
}

// 卡尔曼更新步
void Kalman_Update(KalmanSpeedFilter *kf, float z_measured) {
    // 1. 计算卡尔曼增益: K = P * H^T * (H * P * H^T + R)^-1
    // 简化计算（因为H是[1,0]）
    float S = kf->P[0][0] + kf->R;  // S = H*P*H^T + R
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    
    // 2. 更新状态: x = x + K * (z - H*x)
    float y = z_measured - kf->x[0];  // 测量残差
    kf->x[0] = kf->x[0] + K0 * y;
    kf->x[1] = kf->x[1] + K1 * y;
    
    // 3. 更新协方差: P = (I - K*H) * P
    float P00_new = kf->P[0][0] - K0 * kf->P[0][0];
    float P01_new = kf->P[0][1] - K0 * kf->P[0][1];
    float P10_new = kf->P[1][0] - K1 * kf->P[0][0];
    float P11_new = kf->P[1][1] - K1 * kf->P[0][1];
    
    kf->P[0][0] = P00_new;
    kf->P[0][1] = P01_new;
    kf->P[1][0] = P10_new;
    kf->P[1][1] = P11_new;
}
//
// @简介：更新函数，以dt周期调用
// 
float Kalman_clc(KalmanSpeedFilter *kf, float raw_speed) {
    // 预测步（假设每次调用时间间隔固定）
    Kalman_Predict(kf);
    // 更新步（使用测量值）
    Kalman_Update(kf, raw_speed);
    // 返回滤波后的速度
    return kf->x[0];
}
