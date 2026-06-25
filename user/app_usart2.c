#include "module.h"
#include "app_usart2.h"

//
// @简介：对串口2进行初始化，串口2作为平衡车的调试接口
//
void App_USART3_Init(void)
{
	// #1. 对PB10和PB11进行初始化，PB10 - Tx - AF_PP，PB11 - Rx - IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 开启GPIOA的时钟
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	// PB10 - Tx - AF_PP
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; // 921,600
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// PB11 - Rx - IPU
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// #2. 开启USART3的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	// #3. 初始化USART3的参数
	USART_InitTypeDef USART_InitStruct = {0};
	
	USART_InitStruct.USART_BaudRate = 921600;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USART3, &USART_InitStruct);
    
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USARTz Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //开启接收中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	// #4. 闭合USART2的总开关
	USART_Cmd(USART3, ENABLE);
}
driver_init("USART3_Init", App_USART3_Init);                     /*串口初始化*/

