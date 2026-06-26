#include "math.h"
#include "app_pwm.h"
#include "ai_pwm.h"
#include "module.h"

    /**TIM3 GPIO Configuration    
    bin1-> PA6          ------> TIM3_CH1
    ain1-> PA7          ------> TIM3_CH2 
    ain2-> PA3-> B0     ------> TIM3_CH3 
	bin2-> PA4-> B1     ------> TIM3_CH4
    */

#define MOTOR_INIT_PERCENT     	30      //起始30%
#define MOTOR_MAX_VOLTAGE      	90     //按照9V调整,放大10倍
#define MOTOR_MAX_PERIOD 		800

ai_pwm_t MotorLeftSoftForward;	//左轮前进
ai_pwm_t MotorLeftSoftBack;		//左轮后退
ai_pwm_t MotorRightSoftForward;	//右轮前进
ai_pwm_t MotorRightSoftBack;	//右轮后退
//
// @简介：控制TB6612进入休眠状态或者活动状态
// @参数：on    0 - 休眠状态，向STBY写L
//           非零 - 活动状态，向STBY写H
//
void App_PWM_Cmd(uint8_t on)
{
	if(on == 0)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET); // 休眠
	}
	else
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET); // 休眠
	}
}

// @简介：设置左电机的占空比,无软启动
// @参数：duty - 占空比的具体值，范围-100.0f ~ +100.0f
// @note: duty > 0 左轮胎前进，反之后退
void App_PWM_Set_L(float Duty)
{
	float sign; // 符号，正数 - +1， 负数 - -1
	
	if(Duty >= 0) sign = 1;
	else sign = -1;
	
	Duty = fabsf(Duty);
	uint16_t ccr = Duty / 100.0f * MOTOR_MAX_PERIOD;
	if(sign >= 0) // 前进方向
	{
        TIM_SetCompare2(TIM3, 0);
        TIM_SetCompare3(TIM3, ccr);
	}
	else
	{
        TIM_SetCompare3(TIM3, 0);
        TIM_SetCompare2(TIM3, ccr);
	}
}

// @简介：设置右电机的占空比,无软启动
// @参数：duty - 占空比的具体值，范围-100.0f ~ +100.0f
// @note: duty > 0 右轮胎前进，反之后退
void App_PWM_Set_R(float Duty)
{
	float sign; // 符号，正数 - +1， 负数 - -1
	
	if(Duty >= 0) sign = 1;
	else sign = -1;
	
	Duty = fabsf(Duty);
	uint16_t ccr = Duty / 100.0f * MOTOR_MAX_PERIOD;
	if(sign >= 0) // 前进方向
	{
        TIM_SetCompare1(TIM3, 0);
        TIM_SetCompare4(TIM3, ccr);
	}
	else
	{
        TIM_SetCompare4(TIM3, 0);
        TIM_SetCompare1(TIM3, ccr);
	}
}

// @简介：硬件设置左电机的占空比,有软启动
// @参数：duty - 占空比的具体值，范围-100.0f ~ +100.0f
// @note: duty > 0 左轮胎前进，反之后退
void HW_PWM_Set_L(float percent)
{
    if(percent >= 0)
        motorLeftHwClk(percent);
    else if(percent < 0)
        motorLeftHwUnclk(-percent);
//    else
//        motorLeftHwBreak();
}
// @简介：硬件设置右电机的占空比,有软启动
// @参数：duty - 占空比的具体值，范围-100.0f ~ +100.0f
// @note: duty > 0 右轮胎前进，反之后退
void HW_PWM_Set_R(float percent)
{
    if(percent >= 0)
        motorRightHwClk(percent);
    else if(percent < 0)
        motorRightHwUnclk(-percent);
//    else
//        motorRightHwBreak();
}
//
// @简介：电机硬件初始化
//        AIN1 - PA7 PB0 - Out_PP
//        AIN2 - PA6 PB1 - Out_PP
//
void App_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	// 初始化4路GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//A3 A4硬件跳线连接B0 B1,设置成高阻态
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
    //A6 A7
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	//B0 B1
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
	
    // 初始化PWM
	// 对定时器3进行初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启定时器3的时钟
	
	// 设置时基单元的参数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = MOTOR_MAX_PERIOD - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 9-1;        //10KHZ
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	// 配置4路输出比较
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//    
//    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	// 配置MOE的开关
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	// 闭合定时器的总开关
	TIM_Cmd(TIM3, ENABLE);
}

//左轮硬件设置PWM
void motorLeftHwSetDuty(uint32_t duty)
{
    if(TIM_GetCapture2(TIM3) != duty)
    {
        TIM_SetCompare2(TIM3, duty);
    }
}

void motorLeftHwSetDuty2(uint32_t duty)
{
    if(TIM_GetCapture3(TIM3) != duty)
    {
        TIM_SetCompare3(TIM3, duty);
    }
}

//右轮硬件设置PWM
void motorRightHwSetDuty(uint32_t duty)
{
    if(TIM_GetCapture1(TIM3) != duty)
    {
        TIM_SetCompare1(TIM3, duty);
    }
}

void motorRightHwSetDuty2(uint32_t duty)
{
    if(TIM_GetCapture4(TIM3) != duty)
    {
        TIM_SetCompare4(TIM3, duty);
    }
}
//设置软启目标值
void motorLeftHwSetPercent(uint32_t percent)
{
    aiPwmSetPercent(&MotorLeftSoftForward, percent, motorLeftHwSetDuty);
}

void motorLeftHwSetPercent2(uint32_t percent)
{
    aiPwmSetPercent(&MotorLeftSoftBack, percent, motorLeftHwSetDuty2);
}
void motorRightHwSetPercent(uint32_t percent)
{
    aiPwmSetPercent(&MotorRightSoftForward, percent, motorRightHwSetDuty);
}

void motorRightHwSetPercent2(uint32_t percent)
{
    aiPwmSetPercent(&MotorRightSoftBack, percent, motorRightHwSetDuty2);
}

//硬件PWM进程
void motorHwProcess(void)
{
    aiPwmProcess(&MotorLeftSoftForward, 90, motorLeftHwSetDuty);
    aiPwmProcess(&MotorLeftSoftBack,    90, motorLeftHwSetDuty2);
    aiPwmProcess(&MotorRightSoftForward,90, motorRightHwSetDuty);
    aiPwmProcess(&MotorRightSoftBack,   90, motorRightHwSetDuty2);
}
//后退
void motorLeftHwUnclk(uint8_t percent)
{
    motorLeftHwSetPercent2(0);
    motorLeftHwSetPercent(percent);
    motorHwProcess();
}
//前进
void motorLeftHwClk(uint8_t percent)
{
    motorLeftHwSetPercent(0);
    motorLeftHwSetPercent2(percent);
    motorHwProcess();
}
//停止
void motorLeftHwBreak(void)
{
    motorLeftHwSetPercent(0);
    motorLeftHwSetPercent2(0);
    motorHwProcess();
}

void motorRightHwUnclk(uint8_t percent)
{
    motorRightHwSetPercent2(0);
    motorRightHwSetPercent(percent);
    motorHwProcess();
}
void motorRightHwClk(uint8_t percent)
{
    motorRightHwSetPercent(0);
    motorRightHwSetPercent2(percent);
    motorHwProcess();
}
void motorRightHwBreak(void)
{
    motorRightHwSetPercent(0);
    motorRightHwSetPercent2(0);
    motorHwProcess();
}

void motorHwInit(void){

	App_PWM_Init();
	aiPwmInit(&MotorLeftSoftForward, 	MOTOR_MAX_PERIOD, MOTOR_MAX_VOLTAGE, MOTOR_INIT_PERCENT, MOTOR_INIT_PERCENT);
	aiPwmInit(&MotorLeftSoftBack, 		MOTOR_MAX_PERIOD, MOTOR_MAX_VOLTAGE, MOTOR_INIT_PERCENT, MOTOR_INIT_PERCENT);
	aiPwmInit(&MotorRightSoftForward, 	MOTOR_MAX_PERIOD, MOTOR_MAX_VOLTAGE, MOTOR_INIT_PERCENT, MOTOR_INIT_PERCENT);
	aiPwmInit(&MotorRightSoftBack, 		MOTOR_MAX_PERIOD, MOTOR_MAX_VOLTAGE, MOTOR_INIT_PERCENT, MOTOR_INIT_PERCENT);
	motorLeftHwBreak();
	motorRightHwBreak();
}

float pwmR,pwmL = 0;
void PWM_Test1(void) // main
{
    static int state_count = 0;
	state_count++;
    state_count = state_count % 3;
	
//    App_PWM_Set_L(pwmL);
//    App_PWM_Set_R(pwmR);
    
//    if(pwmL>=0) motorLeftHwClk(pwmL);
//    else        motorLeftHwUnclk(-pwmL);
//    if(pwmR>=0) motorRightHwClk(pwmR);
//    else        motorRightHwUnclk(-pwmR);
    motorHwProcess();
    
}

driver_init("PWM", motorHwInit);           /*电机硬件初始化*/
#ifdef USE_FOST_START
task_register("PWM", motorHwProcess, 0);      /*测试任务, 2s*/
#endif
