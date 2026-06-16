#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include "stm32f10x.h"
#include "Kalman.h"
#include "include.h"

#define filter_none     0       //无滤波
#define filter_kalman   1       //卡尔曼滤波
#define filter_LowPass  2       //一阶低通滤波
#define ALPHA 0.1               //一阶低通绿波系数
#define FILTER_MODE     1       //卡尔曼模式

typedef struct {
    FCT_VOID    hw_init;        // 硬件初始化
    UINT8_FCT   hw_ReadA;       // A相硬件读取
    UINT8_FCT   hw_ReadB;       // B相硬件读取
    int64_t     encoder;        // 电机编码器的值
    int8_t      direction;      // 电机旋转的方向，1 - 正转，-1 - 反转
    uint64_t    t0, t1;     // 电机编码器发生变化的时间，单位us
    float       speed;          //电机当前速度
#if (FILTER_MODE == filter_LowPass)
	float       speed_Last;     //电机上次速度
#elif (FILTER_MODE == filter_kalman)
    KalmanSpeedFilter Kalman;   //kalman滤波矩阵
#endif
} encoder_t;

void App_Encoder_Init(void);
float App_Encoder_GetPos_L(void);
float App_Encoder_GetPos_R(void);
float App_Encoder_GetSpeed_L(void);
float App_Encoder_GetSpeed_R(void);
float Kalman_GetSpeed_R(void);
float Kalman_GetSpeed_L(void);

#endif
