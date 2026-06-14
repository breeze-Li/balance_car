#include "app_pwm.h"
#include "module.h"

    /**TIM3 GPIO Configuration    
    bin1-> PA6          ------> TIM3_CH1
    ain1-> PA7          ------> TIM3_CH2 
    ain2-> PA3-> B0     ------> TIM3_CH3 
	bin2-> PA4-> B1     ------> TIM3_CH4
    */

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

#include "math.h"

//
// @简介：设置左电机的占空比
// @参数：duty - 占空比的具体值，范围-100.0f ~ +100.0f
// @note: duty > 0 左轮胎前进，反之后退
void App_PWM_Set_L(float Duty)
{
	float sign; // 符号，正数 - +1， 负数 - -1
	
	if(Duty >= 0) sign = 1;
	else sign = -1;
	
	Duty = fabsf(Duty);
	uint16_t ccr = (100.0f - Duty) / 100.0f * 999;
	if(sign >= 0) // 前进方向
	{
        TIM_SetCompare2(TIM3, 999);
        TIM_SetCompare3(TIM3, ccr);
	}
	else
	{
        TIM_SetCompare3(TIM3, 999);
        TIM_SetCompare2(TIM3, ccr);
	}
}

//
// @简介：设置右电机的占空比
// @参数：duty - 占空比的具体值，范围-100.0f ~ +100.0f
// @note: duty > 0 右轮胎前进，反之后退
void App_PWM_Set_R(float Duty)
{
	float sign; // 符号，正数 - +1， 负数 - -1
	
	if(Duty >= 0) sign = 1;
	else sign = -1;
	
	Duty = fabsf(Duty);
	uint16_t ccr = (100.0f - Duty) / 100.0f * 999;
	if(sign >= 0) // 前进方向
	{
        TIM_SetCompare1(TIM3, 999);
        TIM_SetCompare4(TIM3, ccr);
	}
	else
	{
        TIM_SetCompare4(TIM3, 999);
        TIM_SetCompare1(TIM3, ccr);
	}
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
	TIM_TimeBaseInitStruct.TIM_Period = 999;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	// 配置4路输出比较
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	// 配置MOE的开关
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	
	// 闭合定时器的总开关
	TIM_Cmd(TIM3, ENABLE);
}
int pwmR,pwmL;
void PWM_Test1(void) // main
{
    static int state_count = 0;
	state_count++;
    state_count = state_count % 3;
	
    App_PWM_Set_L(pwmL);
    App_PWM_Set_R(pwmR);
    
}

driver_init("PWM", App_PWM_Init);           /*电机硬件初始化*/
task_register("PWM", PWM_Test1, 2000);      /*测试任务, 2s*/
