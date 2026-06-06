#include "stm32f10x.h"
#include "bat_test.h"
#include "app_bat.h"
#include "module.h"
#include "app_button.h"
#include "app_pwm.h"
#include "pwm_test.h"
#include "encoder_test.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
    module_task_init();                         /*模块初始化*/
    while (1) {    
        module_task_process();                   /*任务轮询*/
    }
    
//	Encoder_T_Method_Test();
//	Encoder_M_Method_Test();
//	Encoder_Test();
//	Bat_Test();
//	PWM_Test();
//	App_Bat_Init();
//	App_Button_Init();
//	App_PWM_Init();
	
	while(1)
	{
//		App_Bat_Proc();
//		App_Button_Proc();
	}
}
