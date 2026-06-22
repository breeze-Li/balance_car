#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include "stm32f10x.h"
#include "Kalman.h"
#include "filter.h"
#include "include.h"

#define filter_none     0       // 无滤波
#define filter_kalman   1       // 卡尔曼滤波
#define filter_LowPass  2       // 一阶低通滤波
#define ALPHA 0.05              // 一阶低通绿波系数
#define FILTER_MODE     2       // 卡尔曼模式
#if 0
typedef struct {
    FCT_VOID    hw_init;        // 硬件初始化
    UINT8_FCT   hw_ReadA;       // A相硬件读取
    UINT8_FCT   hw_ReadB;       // B相硬件读取
    int64_t     encoder;        // 电机编码器的值
    int8_t      direction;      // 电机旋转的方向，1 - 正转，-1 - 反转
    int8_t      current_dir;    // 当前采样方向
    int8_t      last_confirm_dir;   // 上次已确认的方向
    uint8_t     dir_confirm_count;  // 方向确认计数
    uint8_t     true_change_count;  // 真实跳变次数
    uint64_t    real_time0;        // 实时记录的时间，单位us
    uint64_t    real_time1;        // 实时记录的时间，单位us
    uint64_t    t0, t1;     // 电机编码器发生变化的时间，单位us
    float       speed;          //电机当前速度
#if (FILTER_MODE == filter_LowPass)
	float       speed_Last;     //电机上次速度
#elif (FILTER_MODE == filter_kalman)
    KalmanSpeedFilter Kalman;   //kalman滤波矩阵
#endif
} encoder_t;
#endif
//编码器硬件接口
typedef struct {
    FCT_VOID    hw_init;        // 硬件初始化
    INT16_FCT   hw_ReadCnt;     // 硬件读取计数
    FCT_VOID    hw_ClearCnt;    // 清除计数
} encoder_hw_t;

typedef struct {
    encoder_hw_t hw;
    int16_t      cnt;            // 硬件编码器计数值
    float        delat_T;        // 电机计算间隔
    float        speed;          // 电机当前速度
//    float        speed_Last;     // 电机上次速度，滤波用
    uint64_t     t0, t1;         // 电机编码器发生变化的时间，单位us
    PT1Filter_t  Speed_Filter;   // 一阶低通滤波器
} encoder_t;

void App_Encoder_Init(void);
float App_Encoder_GetPos_L(void);
float App_Encoder_GetPos_R(void);
float App_Encoder_GetSpeed_L(void);
float App_Encoder_GetSpeed_R(void);
float Kalman_GetSpeed_R(void);
float Kalman_GetSpeed_L(void);

#endif
