#include "module.h"
#include "app_encoder.h"
#include "delay.h"

//#define EN_KALMAN_R
//#define EN_KALMAN_L     //左轮卡尔曼绿波开启

#define filter_none     0       //无滤波
#define filter_kalman   1       //卡尔曼滤波
#define filter_LowPass  2       //一阶低通滤波
#define ALPHA 0.1               //一阶低通绿波系数
#define FILTER_MODE     2       //卡尔曼模式

static volatile int64_t encoder_l = 0; // 左电机编码器的值
static volatile int64_t encoder_r = 0; // 右电机编码器的值
static volatile int8_t direction_l = 1; // 左电机旋转的方向，1 - 正转，-1 - 反转
static volatile int8_t direction_r = 1; // 右电机旋转的方向，1 - 正转，-1 - 反转
static volatile uint64_t t0_l = 0, t1_l = 0; // 左电机编码器发生变化的时间，单位us
static volatile uint64_t t0_r = 0, t1_r = 0; // 右电机编码器发生变化的时间，单位us

float speed_L,speed_LL,speed_R,speed_RL;			//left,left_last,right,right_left
KalmanSpeedFilter Kalman_R,Kalman_L;

static void Encoder_L_Init(void); // 左编码器初始化
static void Encoder_R_Init(void); // 右编码器初始化
float Kalman_GetSpeed(KalmanSpeedFilter *kf, float raw_speed);
float Kalman_GetSpeed_R(void);
float Kalman_GetSpeed_L(void);

//
// @简介：对编码器模块进行初始化
//
void App_Encoder_Init(void)
{
    Kalman_Init(&Kalman_R, 0.001f);
    Kalman_Init(&Kalman_L, 0.001f);
	Encoder_L_Init(); 
	Encoder_R_Init(); 
}

//
// @简介：读取左轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_L(void)
{
	return encoder_l / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
}

//
// @简介：读取右轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_R(void)
{
	return encoder_r / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
}

//
// @简介：读取左轮胎旋转的角速度，omega的值，单位是 度/s
// 
float App_Encoder_GetSpeed_L(void)
{
	__disable_irq(); // 关闭单片机的总中断
	
	int8_t direction_cpy = direction_l;
	uint64_t t0_cpy = t0_l;
	uint64_t t1_cpy = t1_l;
	
	__enable_irq(); // 开启单片机的总中断
	
	if(direction_cpy == +2 || direction_cpy == -2)
	{
#if (FILTER_MODE == filter_LowPass)
		speed_LL = 0;
#endif
		speed_L = 0.0f;
	}
    else
    {
        uint64_t now = GetUs();
        
        float T;
        
        if(t0_cpy - t1_cpy > now - t0_cpy)
        {
            T = (t0_cpy - t1_cpy) * 1.0e-6f;
        }
        else
        {
            T = (now - t0_cpy) * 1.0e-6f;
        }
        speed_L = direction_cpy / T / 22.0f / (30613.0f / 1500.0f) * 360.0f;
    }
#if (FILTER_MODE == filter_LowPass)
    #warning "FILTER_MODE is LOWPASS"
	speed_L = ALPHA * speed_L + (1-ALPHA)*speed_LL;
	speed_LL = speed_L;
#elif (FILTER_MODE == filter_kalman)
    #warning "FILTER_MODE is KALMAN"
    speed_L = Kalman_clc(&Kalman_L, speed_L);
#endif
	return speed_L;
}

//
// @简介：读取右轮胎旋转的角速度，omega的值，单位是 度/s
// 
float App_Encoder_GetSpeed_R(void)
{
	__disable_irq(); // 关闭单片机的总中断
	
	int8_t direction_cpy = direction_r;
	uint64_t t0_cpy = t0_r;
	uint64_t t1_cpy = t1_r;
	
	__enable_irq(); // 开启单片机的总中断
	
	if(direction_cpy == +2 || direction_cpy == -2)
	{
#if (FILTER_MODE == filter_LowPass)
		speed_RL = 0;
#endif
		speed_R = 0.0f;
	}
    else
    {
        uint64_t now = GetUs();
        
        float T;
        
        if(t0_cpy - t1_cpy > now - t0_cpy)
        {
            T = (t0_cpy - t1_cpy) * 1.0e-6f;
        }
        else
        {
            T = (now - t0_cpy) * 1.0e-6f;
        }
        
        speed_R = direction_cpy / T / 22.0f / (30613.0f / 1500.0f) * 360.0f;
    }
#if (FILTER_MODE == filter_LowPass)
	speed_R = ALPHA * speed_R + (1-ALPHA)*speed_RL;
	speed_RL = speed_R;
#elif (FILTER_MODE == filter_kalman)
    speed_R = Kalman_clc(&Kalman_R, speed_R);
#endif
    return speed_R;
}

//
// @简介：左编码器初始化
//
static void Encoder_L_Init(void)
{
	// 初始化A和B的引脚
	// PB6, PB7 - IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	// EXTI初始化
	// 让EXTI_Line6监控PB6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
	
	// 配置EXTI的参数
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line6;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	
	EXTI_Init(&EXTI_InitStruct);
	
	// 开启EXTI的中断
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;// 中断编号
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStruct);
}

//
// @简介：右编码器初始化
//
static void Encoder_R_Init(void)
{
	// 初始化A和B的引脚
	// 关闭JTAG，开启SWD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	// PA1, PA0 - IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// EXTI初始化
	// 让EXTI_Line0监控PA0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	
	// 配置EXTI的参数
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	
	EXTI_Init(&EXTI_InitStruct);
	
	// 开启EXTI的中断
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;// 中断编号
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStruct);
}

//
// @简介：EXTI3的中断响应函数，对应右编码器的A相
//
void EXTI0_IRQHandler(void)
{
	EXTI_ClearFlag(EXTI_Line0); // 对中断标志位清零
	
	t1_r = t0_r;
	t0_r = GetUs();
	
	uint8_t a = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0); // A相的当前电压
	uint8_t b = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1); // B相的当前电压
	
	if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET)) // 现在轮胎正转
	{
		encoder_r++;
		
		if(direction_r < 0) // 之前轮胎是反转
		{
			direction_r = +2;
		}
		else
		{
			direction_r = 1;
		}
	}
	else // 现在轮胎是反转
	{
		encoder_r--;
		
		if(direction_r > 0) // 之前轮胎是正转
		{
			direction_r = -2;
		}
		else
		{
			direction_r = -1;
		}
	}
}

//
// @简介：EXTI15_10的中断响应函数，对应左编码器的A相
//
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line6) == SET)
	{
		EXTI_ClearFlag(EXTI_Line6); // 对标志位进行清零
		
		t1_l = t0_l;
		t0_l = GetUs();
		
		uint8_t a = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6); // A相的当前电压
		uint8_t b = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7); // B相的当前电压
		
		if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET)) // 现在轮胎反转
		{
			encoder_l--;
			
			if(direction_l > 0) // 之前轮胎是正转
			{
				direction_l = -2;
			}
			else
			{
				direction_l = -1;
			}
		}
		else // 现在轮胎是正转
		{
			encoder_l++;
			
			if(direction_l < 0) // 之前轮胎是反转，现在轮胎是正转
			{
				direction_l = +2;
			}
			else
			{
				direction_l = 1;
			}
		}
	}
}
driver_init("Encoder", App_Encoder_Init);                     /*编码器初始化*/
