//vofa接收数据解析

#include "vofa_uart.h"
#include "usart.h"
#include "module.h"
#include "include.h"
#include "app_pwm.h"
#include "app_motor.h"

typedef struct{
    char     paracmdType;   //命令类型
    char     paracmdID;     //命令ID
    void    (*paracmdFunc)(vofaCommand *); //命令函数指针
}paracmd;

//W1 - 左电机占空比
void vofaSetLeftDuty(vofaCommand *cmd)
{
    HW_PWM_Set_L(cmd->floatData);
}

//W2 - 右电机占空比
void vofaSetRightDuty(vofaCommand *cmd)
{
    HW_PWM_Set_R(cmd->floatData);
}

//M1 - 左电机速度,使用PID,单位是rad/s
void vofaSetLeftSpeed(vofaCommand *cmd)
{
    App_Motor_SetOmega_L(cmd->floatData);
}

//M2 - 右电机速度,使用PID,单位是rad/s
void vofaSetRightSpeed(vofaCommand *cmd)
{
    App_Motor_SetOmega_R(cmd->floatData);
}
paracmd paracmdList[]={
    {'W', '1', vofaSetLeftDuty},  //硬件设置左电机占空比 PWM1
    {'W', '2', vofaSetRightDuty}, //硬件设置右电机占空比 PWM2
    {'M', '1', vofaSetLeftSpeed}, //左电机速度,使用PID,单位是rad/s MOTOR1
    {'M', '2', vofaSetRightSpeed}, //右电机速度,使用PID,单位是rad/s MOTOR2
}; //命令列表

//串口命令解析函数
void vofaComParse(void)
{
    //判断是否收到命令帧
    vofaCommand* cmd = (vofaCommand*)vofaCommandParse();
    if(cmd == NULL) return;
    //判断命令类型和ID是否匹配
    if(cmd->cmdType == INVALID || cmd->cmdID == INVALID) return;
    //判断命令函数是否存在
    for(int i = 0; i < dim(paracmdList); i++)
    {
        if(cmd->cmdType == paracmdList[i].paracmdType && cmd->cmdID == paracmdList[i].paracmdID)
        {
            paracmdList[i].paracmdFunc(cmd);
            break;
        }
    }
}
task_register("vofaCommandParse", vofaComParse, 10);      /*测试任务, 2s*/

