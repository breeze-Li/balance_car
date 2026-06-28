#include "include.h"
#include "app_encoder.h"
#include "delay.h"
#include "vofa_uart.h"

#define DELAT_T 10      //运行时间间隔 单位MS

//  volatile encoder_t encoder_R;    //右轮电机编码器实例
//  volatile encoder_t encoder_L;    //左轮电机编码器实例
encoder_t encoder_R;//右轮电机编码器实例
encoder_t encoder_L;//左轮电机编码器实例

#if ENCODER_VOFA_SEND_EN

vofaJustFloatFrame JustFloat_Encoder;

#endif

static void Encoder_L_Init(void); // 左编码器硬件初始化
static void Encoder_R_Init(void); // 右编码器硬件初始化
static void App_EncoderInst_Init(encoder_t *this, FCT_VOID hwInit, INT16_FCT read, FCT_VOID clr);
static int16_t encoder_R_Read(void);
static void encoder_R_clr(void);
static int16_t encoder_L_Read(void);
static void encoder_L_clr(void);

//
// @简介：对编码器模块进行初始化
//
void App_Encoder_Init(void)
{
    App_EncoderInst_Init(&encoder_R, Encoder_R_Init, encoder_R_Read, encoder_R_clr);
    App_EncoderInst_Init(&encoder_L, Encoder_L_Init, encoder_L_Read, encoder_L_clr);
}

//简介：初始化编码器对象 
//参数：对象 硬件初始化 读取计数 清除计数
static void App_EncoderInst_Init(encoder_t *this, FCT_VOID hwInit, INT16_FCT read, FCT_VOID clr)
{
    if(hwInit == NULL) return;
    hwInit();
    this->hw.hw_ReadCnt  = read;
    this->hw.hw_ClearCnt = clr;
    
    this->cnt        = 0;
    this->t0         = 0;
    this->t1         = 0;
	this->speed      = 0;
//	this->speed_Last = 0;
    //截止频率需要调试，使得滤波效果和响应速度折中 目前相应速度在450MS左右
    PT1Filter_InitWithFreq(&this->Speed_Filter, 2, 1000 / DELAT_T);
}
//
static int16_t encoder_R_Read(void){
    return TIM_GetCounter(TIM4);
}

static void encoder_R_clr(void){
    TIM_SetCounter(TIM4, 0);
}

static int16_t encoder_L_Read(void){
    return TIM_GetCounter(TIM2);
}

static void encoder_L_clr(void){
    TIM_SetCounter(TIM2, 0);
}
//
// @简介：读取左轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_L(void)
{
	return encoder_L.cnt / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
}

//
// @简介：读取右轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_R(void)
{
	return encoder_R.cnt / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
}

//
// @简介：读取轮胎旋转的角速度，omega的值，单位是 度/s
// @参数：编码器对象指针
static float App_Encoder_GetSpeed(encoder_t *this)
{
	this->t0 = GetUs();
    this->cnt = this->hw.hw_ReadCnt();
    this->hw.hw_ClearCnt();
    //计算时间间隔
    this->delat_T  = (this->t0 - this->t1) * 1.0e-6f;
    //4倍频，一圈线数为44  *2pi转为弧度制
    this->speed = this->cnt / this->delat_T / 44.0f / (30613.0f / 1500.0f) * 6.2831853f;
    //一阶滤波
//    My_USART_Printf(MY_USART, "%f,", this->speed);
//    this->speed = ALPHA * this->speed + (1.0f - ALPHA) * this->speed_Last;
    this->speed = PT1Filter_Apply(&this->Speed_Filter, this->speed);
    //记录当前速度
//    My_USART_Printf(MY_USART, "%f\n", this->speed);
//    this->speed_Last = this->speed;
    //记录当次时间，为下次计算做准备
	this->t1 = this->t0;
    
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

// 初始化定时器2为编码器模式 左电机
void Encoder_L_Init(void)
{
    // GPIO配置
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 复用功能
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    // 定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 编码器模式配置
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, 
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    // 配置输入滤波（关键！）
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 8;  // 8个采样周期滤波
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

// 初始化定时器2为编码器模式 右电机
void Encoder_R_Init(void)
{
    // 1. 使能GPIOB和TIM4的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 复用功能
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    // 2. 配置PB6和PB7为复用推挽输入 (浮空输入)
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;      // 上拉输入，增强抗干扰
    // 如果有外部上拉电阻也可以配置为 GPIO_Mode_IN_FLOATING
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 3. 配置TIM4为编码器模式
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;          // 自动重载值，最大范围 (0-65535)
    TIM_TimeBaseStruct.TIM_Prescaler = 0;            // 不分频，直接使用72MHz (实际编码器计数不受影响)
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);
    
    // 4. 配置编码器接口模式 (TIM_EncoderMode_TI12 表示同时在TI1和TI2的边沿计数，即4倍频)
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, 
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    // 5. 配置输入滤波 (强烈推荐！)
    // 对通道1 (PB6) 和 通道2 (PB7) 设置相同的滤波参数
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct); // 先填入默认值
    
    // 配置通道1的滤波
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = 0x8;   // 滤波值 0x0~0xF，值越大滤波越强，推荐0x6~0x8
    TIM_ICInit(TIM4, &TIM_ICInitStruct);
    
    // 配置通道2的滤波
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = 0x8;   // 与通道1一致
    TIM_ICInit(TIM4, &TIM_ICInitStruct);
    
    // 6. 清除计数器并开始计数
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

//
// @简介：T法测速的测试代码
//        通过串口把T法测速的Omega值发送到Vofa显示
//
un_floate omega_encoder_l,omega_encoder_r;
void Encoder_T_Method_Test(void)
{
	
    omega_encoder_l = (un_floate)App_Encoder_GetSpeed_L();
    
#if ENCODER_VOFA_SEND_EN
    
    vofaSetJustFloat(&JustFloat_Encoder, omega_encoder_l.hvalve, TxChannel_1, 1);
    vofaSendJustFloat(&JustFloat_Encoder);
    
#endif

//    omega_encoder_r = App_Encoder_GetSpeed_R();
//    My_USART_Printf(MY_USART, "%f,%f\n", omega_encoder_l, omega_encoder_r);//firewater格式
}

//task_register("key", Encoder_T_Method_Test, DELAT_T);          /*T法测试任务, 1KHZ*/
driver_init("Encoder", App_Encoder_Init);                     /*编码器初始化*/
