#include "app_motor.h"
#include "pid.h"
#include "task.h"
#include "app_encoder.h"
#include "app_pwm.h"
#include "app_bat.h"
#include "include.h"

PID_TypeDef pid_motor_l; // 左电机调速系统的PID控制器
PID_TypeDef pid_motor_r; // 右电机调速系统的PID控制器

#define KP 0.4f
#define KI 7.0f
#define KD 0 //0.01

//
// @简介：初始化左右电机调速系统
//
void App_Motor_Init(void)
{
	PID_Init(&pid_motor_l, KP, KI, KD);
	PID_LimitConfig(&pid_motor_l, +8.4f, -8.4f);
	PID_Init(&pid_motor_r, KP, KI, KD);
	PID_LimitConfig(&pid_motor_r, +8.4f, -8.4f);
}

//
// @简介：电机调速系统的进程函数
//
float omega_l,omega_r;
void App_Motor_Proc(void)
{
//	PERIODIC(1) // 每间隔1ms执行一次
	
	// #1. 通过编码器获取左右电机旋转的角速度
	omega_l = App_Encoder_GetSpeed_L();
	omega_r = App_Encoder_GetSpeed_R();
	
	// #2. 计算PID控制器的输出
	float ua_l = PID_Compute(&pid_motor_l, omega_l);
	float ua_r = PID_Compute(&pid_motor_r, omega_r);
	
	// #3. 将电压Ua设置到电机两端
	// 获取电池电压vbat
	float vbat = App_Bat_Get();
	
	// 计算左右电机PWM信号的占空比
	float duty_l = ua_l / vbat * 100.0f;
	float duty_r = ua_r / vbat * 100.0f;
	
	App_PWM_Set_L(duty_l);
	App_PWM_Set_R(duty_r);
}

//
// @简介：用来设置左电机的转速Omega的值
// @参数：Omega - 表示电机的转速，单位是rad/s
//
void App_Motor_SetOmega_L(float Omega)
{
	PID_ChangeSP(&pid_motor_l, Omega);
}

//
// @简介：用来设置右电机的转速Omega的值
// @参数：Omega - 表示电机的转速，单位是rad/s
//
void App_Motor_SetOmega_R(float Omega)
{
	PID_ChangeSP(&pid_motor_r, Omega);
}

//
// @简介：开关电机
// @参数 On：控制电机的开关，0 - 关闭 非零 - 开启
//
void App_Motor_Cmd(uint8_t On)
{
	App_PWM_Cmd(On);
	// 在开关电机的同时需要对PID控制器进行复位
	PID_Reset(&pid_motor_l);
	PID_Reset(&pid_motor_r);
}


float targetOmega = 0.0f;
void motot_pid_test()
{
//    targetOmega = (GetTick() / 1000) % 6 * 3.0f;
    
    App_Motor_SetOmega_L(targetOmega);
    App_Motor_SetOmega_R(targetOmega);
    
    App_Motor_Proc();
    My_USART_Printf(MY_USART, "%.3f,%.3f,%.3f\n", targetOmega, omega_l, omega_r);
}

driver_init("Motor", App_Motor_Init);                     /*电机pid初始化*/
//task_register("Motor", motot_pid_test, 5);            /*T法测试任务, 1KHZ*/
