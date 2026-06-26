#ifndef APP_PWM_H
#define APP_PWM_H

#include "stm32f10x.h"

#define USE_FOST_START

void App_PWM_Init(void);
void App_PWM_Cmd(uint8_t on);
void App_PWM_Set_L(float Duty);
void App_PWM_Set_R(float Duty);
void HW_PWM_Set_L(float percent);
void HW_PWM_Set_R(float percent);

void motorLeftHwClk(uint8_t);
void motorLeftHwUnclk(uint8_t);
void motorLeftHwBreak(void);
void motorRightHwClk(uint8_t);
void motorRightHwUnclk(uint8_t);
void motorRightHwBreak(void);

#endif
