#ifndef KALAMAN
#define KALAMAN

// Kalman滤波器结构体
typedef struct {
    // 状态向量 x = [速度, 加速度]
    float x[2];
    // 协方差矩阵 P (2x2)
    float P[2][2];
    // 状态转移矩阵 A (2x2)
    float A[2][2];
    // 观测矩阵 H (1x2)
    float H[1][2];
    // 过程噪声协方差 Q (2x2)
    float Q[2][2];
    // 测量噪声协方差 R (1x1)
    float R;
    // 时间步长 dt (秒)
    float dt;
} KalmanSpeedFilter;

// 初始化滤波器
void Kalman_Init(KalmanSpeedFilter *kf, float dt);
// 卡尔曼预测步
void Kalman_Predict(KalmanSpeedFilter *kf);
// 卡尔曼更新步
void Kalman_Update(KalmanSpeedFilter *kf, float z_measured);
float Kalman_clc(KalmanSpeedFilter *kf, float raw_speed);


#endif
