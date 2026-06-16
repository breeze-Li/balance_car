#include "include.h"
#include "app_encoder.h"
#include "delay.h"

static volatile encoder_t encoder_R;    //右轮电机编码器实例
static volatile encoder_t encoder_L;    //左轮电机编码器实例

static void Encoder_L_Init(void); // 左编码器硬件初始化
static void Encoder_R_Init(void); // 右编码器硬件初始化
static void App_EncoderInst_Init(volatile encoder_t *this, FCT_VOID hwInit, UINT8_FCT readA, UINT8_FCT readB);
static uint8_t encoder_R_ReadA(void);
static uint8_t encoder_R_ReadB(void);
static uint8_t encoder_L_ReadA(void);
static uint8_t encoder_L_ReadB(void);

//
// @简介：对编码器模块进行初始化
//
void App_Encoder_Init(void)
{
    App_EncoderInst_Init(&encoder_R, Encoder_R_Init, encoder_R_ReadA, encoder_R_ReadB);
    App_EncoderInst_Init(&encoder_L, Encoder_L_Init, encoder_L_ReadA, encoder_L_ReadB);
}

//简介：初始化编码器对象 
//参数：对象 硬件初始化 A相读取 B相读取
static void App_EncoderInst_Init(volatile encoder_t *this, FCT_VOID hwInit, UINT8_FCT readA, UINT8_FCT readB)
{
    if(hwInit == NULL) return;
    hwInit();
    this->hw_ReadA  = readA;
    this->hw_ReadB  = readB;
    
    this->encoder   = 0;
    this->direction = 1;
    this->t0        = 0;
    this->t1        = 0;
#if (FILTER_MODE    == filter_LowPass)
	this->speed     = 0;
	this->speed_Last = 0;
#elif (FILTER_MODE == filter_kalman)
    Kalman_Init(&this->Kalman, 0.001f);
#endif
}

//
// @简介：读取左轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_L(void)
{
	return encoder_L.encoder / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
}

//
// @简介：读取右轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_R(void)
{
	return encoder_R.encoder / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
}

//
// @简介：读取轮胎旋转的角速度，omega的值，单位是 度/s
// @参数：编码器对象指针
static float App_Encoder_GetSpeed(volatile encoder_t *this)
{
	__disable_irq(); // 关闭单片机的总中断
	
	int8_t direction_cpy = this->direction;
	uint64_t t0_cpy = this->t0;
	uint64_t t1_cpy = this->t1;
	
	__enable_irq(); // 开启单片机的总中断
	
	if(direction_cpy == +2 || direction_cpy == -2)
	{
#if (FILTER_MODE == filter_LowPass)
		this->speed_Last = 0;
#endif
		this->speed = 0.0f;
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
        this->speed = direction_cpy / T / 22.0f / (30613.0f / 1500.0f) * 360.0f;
    }
#if (FILTER_MODE == filter_LowPass)
    #warning "FILTER_MODE is LOWPASS"
	this->speed = ALPHA * this->speed + (1-ALPHA)*this->speed_Last;
	this->speed_Last = this->speed;
#elif (FILTER_MODE == filter_kalman)
    #warning "FILTER_MODE is KALMAN"
    this->speed = Kalman_clc(&this->Kalman, this->speed);
#endif
	return this->speed;
}

//
// @简介：读取左轮胎旋转的角速度，omega的值，单位是 度/s
//
float App_Encoder_GetSpeed_L(void)
{
    return App_Encoder_GetSpeed(&encoder_L);
}
//
// @简介：读取右轮胎旋转的角速度，omega的值，单位是 度/s
// 
float App_Encoder_GetSpeed_R(void)
{
    //电机对称安装，其中一项要取反，才能保证两轮的前进和后退一致。按自己定义的来
    return -App_Encoder_GetSpeed(&encoder_R);
}

//
// @简介：左编码器硬件初始化
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
// @简介：右编码器硬件初始化
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


static uint8_t encoder_R_ReadA(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0); // A相的当前电压
}
static uint8_t encoder_R_ReadB(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1); // B相的当前电压
}
static uint8_t encoder_L_ReadA(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6); // A相的当前电压
}
static uint8_t encoder_L_ReadB(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7); // B相的当前电压
}


//简介：编码器中断回调函数
//参数：编码器对象指针
void Encoder_Irq(volatile encoder_t *this)
{
    this->t1 = this->t0;
	this->t0 = GetUs();
	
	uint8_t a = this->hw_ReadA(); // A相的当前电压
	uint8_t b = this->hw_ReadB(); // B相的当前电压
	
	if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET)) // 现在轮胎正转
	{
		this->encoder--;
		
		if(this->direction > 0) // 之前轮胎是正转
		{
			this->direction = -2;
		}
		else
		{
			this->direction = -1;
		}
	}
	else // 现在轮胎是反转
	{
        this->encoder++;
		
		if(this->direction < 0) // 之前轮胎是反转
		{
			this->direction = +2;
		}
		else
		{
			this->direction = 1;
		}
	}
}

//
// @简介：EXTI3的中断响应函数，对应右编码器的A相
//
void EXTI0_IRQHandler(void)
{
	EXTI_ClearFlag(EXTI_Line0); // 对中断标志位清零
	
    Encoder_Irq(&encoder_R);
}

//
// @简介：EXTI15_10的中断响应函数，对应左编码器的A相
//
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line6) == SET)
	{
		EXTI_ClearFlag(EXTI_Line6); // 对标志位进行清零
        
		Encoder_Irq(&encoder_L);
	}
}
//
// @简介：T法测速的测试代码
//        通过串口把T法测速的Omega值发送到Vofa显示
//
void Encoder_T_Method_Test(void)
{
//	App_USART3_Init();
//	App_Encoder_Init();
	float omega_l,omega_r;
//	while(1)
//	{
//		Delay(1);
//        omega_r=Kalman_GetSpeed_R();
//        omega_l=Kalman_GetSpeed_L();
		omega_l = App_Encoder_GetSpeed_L();
		omega_r = App_Encoder_GetSpeed_R();
		
		My_USART_Printf(USART3, "%f,%f\n", omega_l, omega_r);
//	}
}

task_register("key", Encoder_T_Method_Test, 10);          /*T法测试任务, 1KHZ*/
driver_init("Encoder", App_Encoder_Init);                     /*编码器初始化*/
