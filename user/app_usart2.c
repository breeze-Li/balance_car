#include "module.h"
#include "app_usart2.h"
#include "fifo.h"
#include "include.h"

//USART3-tx->CH2, rx->CH3

#define USARTy_Tx_DMA_Irq        DMA1_Channel2_IRQn
#define USARTy_Rx_DMA_Irq        DMA1_Channel3_IRQn
#define USARTy_Tx_DMA_Channel    DMA1_Channel2
#define USARTy_Rx_DMA_Channel    DMA1_Channel3
#define USARTy_Tx_DMA_FLAG       DMA1_IT_TC2
#define USARTy_Rx_DMA_FLAG       DMA1_IT_TC3
#define USARTy_DR_Base           (uint32_t)&USART3->DR

#define BUFF_SIZE_TX                200
#define BUFF_SIZE_RX                200
#define FIFO_SIZE_TX                200
#define FIFO_SIZE_RX                200

struct
{
    fifo_t  fifoRx;
    fifo_t  fifoTx;
    timer_t tTxTick;
    timer_t tDelayToogle;
    timer_t tDelayTx;
    uint8_t sendBusy;
    uint8_t bDelayToogle;
    uint8_t bDelayTx;
    //解析缓存
    uint8_t aby_FIFOBuffRx[FIFO_SIZE_RX];
    uint8_t aby_FIFOBuffTx[FIFO_SIZE_TX];
    //硬件缓存
    uint8_t HwbuffTx[BUFF_SIZE_TX];
    uint8_t HwbuffRx[BUFF_SIZE_RX];
} comHw;

void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* USARTy_Tx_DMA_Channel (triggered by USARTy Tx event) Config */
    DMA_DeInit(USARTy_Tx_DMA_Channel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = USARTy_DR_Base;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)comHw.HwbuffTx;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_TX;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(USARTy_Tx_DMA_Channel, &DMA_InitStructure);
    
    
    // === DMA接收通道 (DMA1_Channel2) USARTy_Rx_DMA_Channel
    DMA_DeInit(USARTy_Rx_DMA_Channel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)comHw.HwbuffRx;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_RX;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(USARTy_Rx_DMA_Channel, &DMA_InitStructure);
    // 使能DMA接收
    DMA_ITConfig(USARTy_Rx_DMA_Channel, DMA_IT_TC, ENABLE);
    DMA_Cmd(USARTy_Rx_DMA_Channel, ENABLE);
    NVIC_EnableIRQ(USARTy_Tx_DMA_Irq);  // 发送
    NVIC_EnableIRQ(USARTy_Rx_DMA_Irq);  // 接收
}

//
// @简介：对串口2进行初始化，串口2作为平衡车的调试接口
//
void App_USART3_Init(void)
{
    DMA_Configuration();
    
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
    //开启接收中断，空闲中断
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    //关联DMA
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  // 发送也需要启用
	// #4. 闭合USART2的总开关
	USART_Cmd(USART3, ENABLE);
    
    memset(&comHw, 0, sizeof(comHw));
    TICK_RESET(comHw.tTxTick);
    TICK_RESET(comHw.tDelayToogle);
    fifoInit(&comHw.fifoRx, comHw.aby_FIFOBuffRx, sizeof(comHw.aby_FIFOBuffRx));
    fifoInit(&comHw.fifoTx, comHw.aby_FIFOBuffTx, sizeof(comHw.aby_FIFOBuffTx));
}

void comHwTxInt(void)
{
    //发送完成后关闭发送DMA
    DMA_Cmd(USARTy_Tx_DMA_Channel, DISABLE);
    DMA_ITConfig(USARTy_Tx_DMA_Channel, DMA_IT_TC, DISABLE);

    comHw.sendBusy = false;
    comHw.bDelayToogle = true;
    TICK_RESET(comHw.tDelayToogle);
    TICK_RESET(comHw.tTxTick);
}
uint8_t count = 0;
void comHwRxInt(void)
{
    // 关闭DMA传输 
    DMA_Cmd(USARTy_Rx_DMA_Channel, DISABLE);
    count = DMA_GetCurrDataCounter(USARTy_Rx_DMA_Channel);
    fifoEnqueue(&comHw.fifoRx, comHw.HwbuffRx, dim(comHw.HwbuffRx) - DMA_GetCurrDataCounter(USARTy_Rx_DMA_Channel)); /* 转存数据到待处理数据缓冲区*/
    
    // 4. 重新配置DMA：重置计数器 + 恢复内存地址偏移
    USARTy_Rx_DMA_Channel->CNDTR = BUFF_SIZE_RX;
    USARTy_Rx_DMA_Channel->CMAR = (uint32_t)comHw.HwbuffRx; // 恢复内存基址

    // 5. 重新开启DMA
    DMA_Cmd(USARTy_Rx_DMA_Channel, ENABLE);
}
/**
* @brief 串口中断
*/
void USART3_IRQHandler(void)
{
//    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//        uint8_t byte_data = USART_ReceiveData(USART3);
//        uartCMDRecv(byte_data);
//    }
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        uint8_t byte_data = USART_ReceiveData(USART3); //读DR清除中断标志
        comHwRxInt();
    }
}
//发送完成
void DMA1_Channel2_IRQHandler(void)
{
    if(DMA_GetITStatus(USARTy_Tx_DMA_FLAG))
    {
        DMA_ClearITPendingBit(USARTy_Tx_DMA_FLAG);
        comHwTxInt();
    }
}
//接收完成
void DMA1_Channel3_IRQHandler(void)
{
    if(DMA_GetITStatus(USARTy_Rx_DMA_FLAG))
    {
        DMA_ClearITPendingBit(USARTy_Rx_DMA_FLAG);
        comHwRxInt();
    }

}
void comHwProcess(void)
{
    if(!comHw.sendBusy)
    {
        uint16_t count = fifoPeak(&comHw.fifoTx);
        if(count > 0)
        {
//            UART_DE_TX();
            comHw.sendBusy = true;
            TICK_RESET(comHw.tTxTick);
            count = fifoDequeue(&comHw.fifoTx, comHw.HwbuffTx, sizeof(comHw.HwbuffTx));

            // 4. 重新配置DMA：打开完成中断 + 发送数量
            USARTy_Tx_DMA_Channel->CNDTR = count;
//            USARTy_Tx_DMA_Channel->CMAR = (uint32_t)comHw.HwbuffRx; // 恢复内存基址
            /* enable DMA channel */
            DMA_ITConfig(USARTy_Tx_DMA_Channel, DMA_IT_TC, ENABLE);  // 使能完成中断
            DMA_Cmd(USARTy_Tx_DMA_Channel, ENABLE);
        }
    }

    if((comHw.sendBusy && TICK_COUNTER(comHw.tTxTick, 500)) || (comHw.bDelayToogle && TICK_COUNTER(comHw.tDelayToogle, 3)))
    {
//        UART_DE_RX();
        comHw.sendBusy = false;
        comHw.bDelayToogle = false;
        TICK_RESET(comHw.tTxTick);
        TICK_RESET(comHw.tDelayToogle);
    }
}
//
// @简介：发送缓从区数据设置， 要发送的数据地址，要发送的数据长度
//
uint16_t comHwSend(uint8_t *buffer, uint16_t size)
{
    NVIC_DisableIRQ(USART3_IRQn);
    fifoEnqueue(&comHw.fifoTx, buffer, size);
    NVIC_EnableIRQ(USART3_IRQn);
    return 1;
}

//
// @简介：接收缓从区数据， 要接收的数据地址，要接收的数据长度
//
uint16_t comHwRead(uint8_t *buffer, uint16_t size)
{
    NVIC_DisableIRQ(USART3_IRQn);
    fifoDequeue(&comHw.fifoRx, buffer, size);
    NVIC_EnableIRQ(USART3_IRQn);
    return 1;
}

void DMA_Tx_Test(void)
{
    uint8_t TxBuffer1[] = "USART DMA Interrupt: USARTy -> USARTz using DMA Tx and Rx Flag";
    comHwSend(TxBuffer1, dim(TxBuffer1));
}
driver_init("USART3_Init", App_USART3_Init);                     /*串口初始化*/
task_register("HWUSART", comHwProcess, 0);
task_register("DMA_Tx_Test", DMA_Tx_Test, 100);

