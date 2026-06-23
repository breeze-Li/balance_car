#ifndef __AI_PWM_H__
#define __AI_PWM_H__

#include "include.h"


typedef struct
{
    timer_t  tPwm;
    uint16_t maxPeriod;         //最大PWM周期
    uint16_t maxVoltage;        //电机支持最大电压
    uint8_t  tgtPercent;        //目标百分比
    uint8_t  crtPercent;        //当前百分比
    uint8_t  hardInitPercent;   //为硬件缓启占空比
    uint8_t  logicInitPercent;  //逻辑缓启占空比
    uint8_t  hardAdjTime;       //硬件缓启调节时间
    uint8_t  adjTime;           //正常调节时间
    uint8_t  drvStep;           //控制驱动时间变量
} ai_pwm_t;


/*******************************************
//              使用步骤
//1.新建一个ai_pwm_t对像
//2.aiPwmInit()函数初始化,并将参数输入
//3.使用aiPwmSetPercent()设定目标值
//4.主循环中调用aiPwmProcess().
//说明:1.hardInitPercent为硬件缓启占空比,midPercent为逻辑缓启占空比,硬件缓启动默认每2ms调节一次,可以修改hardAdjTime来修改硬启动调节时间
//     2.此模块正常运行时默认每15ms计算一次. 从0加到100,需1.5秒, 可以通过写变量改adjTime时间
//     3.此模块会根据当前电压值来计算实际输出占空比,达到实际输出电压值不高于
//       设定电压值目标
*******************************************/


/*******************************************
//初始化PWM结构
//参数:*pwmBase, pwm控制对像
//参数:maxPeriod, 最大占空比的周期
//参数:maxVoltage, 电机支持最大电压,数值放大10倍,例始24V,正确输入应为240
//参数:hardInitPercente, 为硬件缓启占空比,值范围0-100
//参数:logicInitPercent, 逻辑缓启占空比,值范围0-100,此值需>=hardInitPercente
//说明:在使用前,必须要使用此函数进行初始化
*******************************************/
void aiPwmInit(ai_pwm_t *pwmBase, uint16_t maxPeriod, uint16_t maxVoltage, uint8_t hardInitPercent, uint8_t logicInitPercent);

/*******************************************
//设定pwm占空比目标值
//参数:*pwmBase, pwm控制对像
//参数:targetPercent, 目标占空比,值范围0-100
//参数:setDuty, 设定duty的函数指针
//说明:无
*******************************************/
void aiPwmSetPercent(ai_pwm_t *pwmBase, uint8_t targetPercent, void(*setDuty)(uint32_t));

/*******************************************
//占空比控制
//参数:*pwmBase, pwm控制对像
//参数:crtVoltage, 当前实际电压值,数值放大10倍,例始24V,正确输入应为240
//参数:setDuty, 设定duty的函数指针
//说明:应放在主循环中调用
*******************************************/
void aiPwmProcess(ai_pwm_t *pwmBase, uint16_t crtVoltage, void(*setDuty)(uint32_t));

/*******************************************
//最大电压补偿控制
//参数:setDuty, 当前设定的PWM duty值
//参数:crtVoltage, 当前实际电压值,数值放大10倍,例始24V,正确输入应为240
//参数:maxPeriod, 最大周期值
//参数:deviceMaxVol, 本设备设定的最大电压值
//返回:占空比值
*******************************************/
uint16_t aiPwmVolCompensate(uint16_t setDuty, uint16_t crtVoltage, uint16_t maxPeriod, uint16_t deviceMaxVol);

/*******************************************
//最大电压补偿控制
//参数:percent, 当前设定的PWM percent值
//参数:crtVoltage, 当前实际电压值,数值放大10倍,例始24V,正确输入应为240
//参数:maxPeriod, 最大周期值
//参数:deviceMaxVol, 本设备设定的最大电压值
//返回:占空比值
*******************************************/
uint16_t aiPwmPercentVolCompensate(uint8_t percent, uint16_t crtVoltage, uint16_t maxPeriod, uint16_t deviceMaxVol);


#endif
