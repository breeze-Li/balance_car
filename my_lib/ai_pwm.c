#include "ai_pwm.h"
#include "include.h"

/*******************************************
//初始化PWM结构
//参数:*pwmBase, pwm控制对像
//参数:maxPeriod, 最大占空比的周期
//参数:maxVoltage, 电机支持最大电压,数值放大10倍,例始24V,正确输入应为240
//参数:hardInitPercente, 为硬件缓启占空比,值范围0-100
//参数:logicInitPercent, 逻辑缓启占空比,值范围0-100,此值需>=hardInitPercente
//说明:在使用前,必须要使用此函数进行初始化
*******************************************/
void aiPwmInit(ai_pwm_t *pwmBase, uint16_t maxPeriod, uint16_t maxVoltage, uint8_t hardInitPercent, uint8_t logicInitPercent)
{
    pwmBase->tPwm            = TICK_GET();
    pwmBase->maxPeriod       = maxPeriod;
    pwmBase->maxVoltage      = maxVoltage;
    pwmBase->tgtPercent      = 0;
    pwmBase->crtPercent      = 0;
    pwmBase->hardInitPercent = hardInitPercent;
    pwmBase->logicInitPercent = logicInitPercent;
    pwmBase->hardAdjTime     = 2;       // 默认每2ms调节一次硬件缓启
    pwmBase->adjTime         = 5;      // 默认每15ms调节一次逻辑缓启
    pwmBase->drvStep         = 0;
}


/*******************************************
//设定pwm占空比目标值
//参数:*pwmBase, pwm控制对像
//参数:targetPercent, 目标占空比,值范围0-100
//参数:setDuty, 设定duty的函数指针
//说明:无
*******************************************/
void aiPwmSetPercent(ai_pwm_t *pwmBase, uint8_t targetPercent, void(*setDuty)(uint32_t))
{
    // 目标值范围限制在0-100
    //限制有效输出,无效值时不输出
    RANGE_SET(targetPercent, 0, 100, 0);

    pwmBase->tgtPercent = targetPercent;

    // 避免未使用参数的编译器警告
    (void)setDuty;
}


/*******************************************
//占空比控制
//参数:*pwmBase, pwm控制对像
//参数:crtVoltage, 当前实际电压值,数值放大10倍,例始24V,正确输入应为240
//参数:setDuty, 设定duty的函数指针
//说明:应放在主循环中调用
*******************************************/
void aiPwmProcess(ai_pwm_t *pwmBase, uint16_t crtVoltage, void(*setDuty)(uint32_t))
{
    uint16_t duty;

    // ---- 硬件缓启阶段: 仅在目标值需要穿越hardInitPercent时生效 ----
    //  条件1: crtPercent < hardInitPercent — 当前值在硬件门槛以下
    //  条件2: tgtPercent >= hardInitPercent — 目标值需要穿越硬件门槛
    //  当目标值本身低于hardInitPercent时(如减速到0),跳过硬件缓启,直接走逻辑调节
    //  注意: 两个阶段必须互斥(else if),否则共享的tPwm会导致TICK_RESET互相破坏计时基准
    if ((pwmBase->crtPercent < pwmBase->hardInitPercent) &&
        (pwmBase->tgtPercent >= pwmBase->hardInitPercent))
    {
        if (TICK_COUNTER(pwmBase->tPwm, pwmBase->hardAdjTime))
        {
            TICK_RESET(pwmBase->tPwm);
            ADD_2_MAX(pwmBase->crtPercent, pwmBase->hardInitPercent);
            pwmBase->drvStep++;
        }
    }
    // ---- 逻辑调节阶段: 从当前值逐步逼近目标值 ----
    else if (pwmBase->crtPercent != pwmBase->tgtPercent)
    {
        if (TICK_COUNTER(pwmBase->tPwm, pwmBase->adjTime))
        {
            TICK_RESET(pwmBase->tPwm);

            if (pwmBase->crtPercent < pwmBase->tgtPercent)
            {
                ADD_2_MAX(pwmBase->crtPercent, pwmBase->tgtPercent);
            }
            else
            {
                DEC_2_MIN(pwmBase->crtPercent, pwmBase->tgtPercent);
            }
            pwmBase->drvStep++;
        }
    }

    // ---- 电压补偿后输出占空比 ----
    duty = aiPwmPercentVolCompensate(pwmBase->crtPercent, crtVoltage,
                                     pwmBase->maxPeriod, pwmBase->maxVoltage);
    setDuty((uint32_t)duty);
}


/*******************************************
//最大电压补偿控制 (基于duty值)
//参数:setDuty, 当前设定的PWM duty值
//参数:crtVoltage, 当前实际电压值,数值放大10倍,例始24V,正确输入应为240
//参数:maxPeriod, 最大周期值
//参数:deviceMaxVol, 本设备设定的最大电压值
//返回:补偿后的占空比值
//说明:当实际电压超过设备最大电压时,按比例减小占空比,
//      使实际输出电压不高于设定电压值
*******************************************/
uint16_t aiPwmVolCompensate(uint16_t setDuty, uint16_t crtVoltage, uint16_t maxPeriod, uint16_t deviceMaxVol)
{
    uint32_t temp;

    // 保护: 电压或周期为零时直接返回原值
    if (crtVoltage == 0 || deviceMaxVol == 0 || maxPeriod == 0)
    {
        return setDuty;
    }

    // 实际电压超过设备最大电压时,按比例缩减占空比
    // 补偿公式: duty_new = duty_old × deviceMaxVol / crtVoltage
    if (crtVoltage > deviceMaxVol)
    {
        temp = (uint32_t)setDuty * deviceMaxVol / crtVoltage;

        if (temp > maxPeriod)
        {
            temp = maxPeriod;
        }

        return (uint16_t)temp;
    }

    return setDuty;
}


/*******************************************
//最大电压补偿控制 (基于百分比值)
//参数:percent, 当前设定的PWM percent值(0-100)
//参数:crtVoltage, 当前实际电压值,数值放大10倍,例始24V,正确输入应为240
//参数:maxPeriod, 最大周期值
//参数:deviceMaxVol, 本设备设定的最大电压值
//返回:补偿后的占空比值
//说明:先将百分比转换为duty值,再进行电压补偿
*******************************************/
uint16_t aiPwmPercentVolCompensate(uint8_t percent, uint16_t crtVoltage, uint16_t maxPeriod, uint16_t deviceMaxVol)
{
    uint32_t duty;

    //限制有效输出,无效值时不输出
    RANGE_SET(percent, 0, 100, 0);
    // 百分比转duty值: duty = maxPeriod × percent / 100
    duty = (uint32_t)maxPeriod * percent / 100;

    return aiPwmVolCompensate((uint16_t)duty, crtVoltage, maxPeriod, deviceMaxVol);
}
