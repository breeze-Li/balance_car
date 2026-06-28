#include <stdio.h>
#include "vofa_uart.h"
#include "usart.h"
#include "module.h"
#include "include.h"

/*VOFA type structure*/
vofaJustFloatFrame JustFloat_Data;
vofaCommand        vofaCommandData;

/**
* @param c: 发送的数据
* @return void
*/
void uartSendByte(const uint8_t c)
{
    My_USART_SendByte(MY_USART, c);     //修改为串口发送接口
}

/**
* @param Array: 发送数据指针
* @param SIZE : 发送数据长度
* @return void
*/
void uartSendData(uint8_t* Array, uint8_t SIZE)
{
    My_USART_SendBytes(MY_USART, Array, SIZE);
}

/**
* @param fdata	：	指向要设置的浮点数据的指针
* @param Channel：	要设置的JustFloat通道号
* @param len	：	要设置的JustFloat数据长度
* @return void
*/
void vofaSetJustFloat(vofaJustFloatFrame* vofaJFFrame, const uint8_t* Puint8data, const TxChannel Channel, const uint32_t len)
{
	//使用16进制拷贝数据
    memcpy(vofaJFFrame->txdata[Channel].hvalve, Puint8data, len * sizeof(float));
//    memcpy(JustFloat_Data.txdata[Channel].hvalve, Puint8data, len * sizeof(float));
}

/**
* @param vofaJFFframe: 包含数据帧的结构体
* @return void
*/
void vofaSendJustFloat(vofaJustFloatFrame* vofaJFFrame)
{
	uartSendData(vofaJFFrame->data, dim(vofaJFFrame->data));
//    uartSendData(JustFloat_Data.data, dim(JustFloat_Data.data));
}

/**
* @param fdata: 指向要发送的浮点数据的指针
* @param ulSize： 要发送的数据个数
* @return void
*/
void vofaSendFirewater(const float* fdata, const uint32_t ulSize)
{
	uint32_t i;
	for (i = 0; i < ulSize - 1; i++)
	{
        My_USART_Printf(MY_USART, "%.6f,", *(fdata + i));
//		printf("%.6f,", *(fdata + i));
	}
    My_USART_Printf(MY_USART, "%.6f,", *(fdata + i));
//	printf("%.6f\n", *(fdata + i));
}

/**
* @param pData: 指向要发送的单字节数据的指针
* @param ulSize： 要发送的数据个数
* @return void
*/
void vofaSendRawdata(uint8_t* pData, const uint32_t ulSize)
{
	uint32_t i;
	for (i = 0; i < ulSize; i++)
	{
		uartSendByte(*(pData + i));
	}
}

/**
* @brief 初始化JustFloat帧结构体
*/
void vofaJustFloatInit(void)
{
	vofaCommandData.cmdID               = INVALID;
	vofaCommandData.cmdType             = INVALID;
	vofaCommandData.completionFlag      = 0;
    vofaCommandData.vofaRxBufferIndex   = 0;
	JustFloat_Data.frametail[0]         = 0x00;
	JustFloat_Data.frametail[1]         = 0x00;
	JustFloat_Data.frametail[2]         = 0x80;
	JustFloat_Data.frametail[3]         = 0x7f;
}

/**
* @brief 将串口收到的数据判断并存入数据包中，并比对帧控制接收完成标志位置位
* @param byte_data： 串口接收到的字节数据 
*/
void uartCMDRecv(uint8_t byte_data) //此函数放在串口中断中
{
	vofaCommandData.uartRxPacket.data[vofaCommandData.vofaRxBufferIndex] = byte_data;

	if (vofaCommandData.uartRxPacket.frametail[0] == '!' && vofaCommandData.uartRxPacket.frametail[1] ==
		'#')
	{
		vofaCommandData.completionFlag = 1;
		vofaCommandData.vofaRxBufferIndex              = 0;
	}
	else if (vofaCommandData.vofaRxBufferIndex > (dim(vofaCommandData.uartRxPacket.data) - 1))
	{
		vofaCommandData.completionFlag      = 0;
		vofaCommandData.vofaRxBufferIndex   = 0;
		memset(vofaCommandData.uartRxPacket.data, 0, dim(vofaCommandData.uartRxPacket.data));
	}
	else
	{
		vofaCommandData.vofaRxBufferIndex++;
	}
}

/**
* @brief 串口接收中断
*/
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t byte_data = USART_ReceiveData(USART3);
        uartCMDRecv(byte_data);
    }
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        uint8_t byte_data = USART_ReceiveData(USART3); //读DR清除中断标志
    }
}
/**
* @brief vofa命令帧解析
*/
vofaCommand* vofaCommandParse(void)
{
    if (vofaCommandData.completionFlag == 1)		//收到命令帧
	{
		vofaCommandData.completionFlag = 0; 			//清楚标志位
		RecFrame* pRxPacket;
		pRxPacket = &vofaCommandData.uartRxPacket;

		//判断帧头帧尾
		if (vofaCommandData.uartRxPacket.framehead[0] != '@' 
			|| vofaCommandData.uartRxPacket.framehead[3] != '=' 
				|| vofaCommandData.uartRxPacket.frametail[RFRAME_TAIL_SIZE - 2] != '!' 
					|| vofaCommandData.uartRxPacket.frametail[RFRAME_TAIL_SIZE - 1] != '#')
		{
			memset(vofaCommandData.uartRxPacket.data, 0, dim(vofaCommandData.uartRxPacket.data));
			return NULL;
		}

		switch (vofaCommandData.uartRxPacket.framehead[1])
		{
			//命令类型,单字节支持A-Z
			case 'A'...'Z': vofaCommandData.cmdType = vofaCommandData.uartRxPacket.framehead[1];
				break;
			default: vofaCommandData.cmdType = INVALID;
				break;
		}

		switch (vofaCommandData.uartRxPacket.framehead[2])
		{
			//命令ID,单字节支持1-9
			case '1'...'9': vofaCommandData.cmdID = vofaCommandData.uartRxPacket.framehead[2];
				break;
			default: vofaCommandData.cmdID = INVALID;
				break;
		}
		vofaCommandData.floatData = pRxPacket->fdata.fvalve;
		//测试JustFloat发送
	    // JustFloat_Data.txdata[0].fvalve  = pRxPacket->fdata.fvalve;
	    // vofaSendJustFloat(&JustFloat_Data);
		pRxPacket = NULL;
		memset(vofaCommandData.uartRxPacket.data, 0, dim(vofaCommandData.uartRxPacket.data));
		return &vofaCommandData;
	}
	return NULL;
}
module_init("vofaJustFloatInit", vofaJustFloatInit);
