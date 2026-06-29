#ifndef VOFA_FUNCTIONS_H__
#define VOFA_FUNCTIONS_H__

#include <stdint.h>
#include <string.h>

//配置vofa 发送
#define ENCODER_VOFA_SEND_EN    0
#define MOTOR_VOFA_SEND_EN      1

#define INVALID 0xFF
#define RFRAME_HEAD_SIZE (4U)
#define RFRAME_TAIL_SIZE (2U)
#define RFRAME_DATA_NUM  (1)    //接收数据数量
#define RFRAME_DATA_SIZE (RFRAME_DATA_NUM * sizeof(float))
#define INCREASE_STEP (0.01f)
#define DECREASE_STEP (0.01f)
#define TFRAME_TAIL_SIZE (4U)

typedef union{
    float   fvalve;     //浮点数
    uint8_t hvalve[4];  //16进制数
}un_floate;

//开启的发送通道数量
typedef enum TxChannel
{
    TxChannel_1 = 0,
    TxChannel_2,
    TxChannel_3,
    TxChannel_4,
    TxChannel_5,
    TxChannel_6,
    TxChannel_7,
    TxChannel_8,
    TxChannel_Total
}TxChannel;

enum CommandID
{
	Direct_Assignment,
	Increase,
	Decrease
};

enum CommandType
{
	Speed = 1,
	Position
};

typedef union
{
    uint8_t         data[TxChannel_Total * sizeof(float) + TFRAME_TAIL_SIZE];
    struct{
        un_floate   txdata[TxChannel_Total];
        uint8_t     frametail[TFRAME_TAIL_SIZE];
    };
} vofaJustFloatFrame;

//接收数据结构
typedef union
{
    uint8_t     data[RFRAME_DATA_SIZE + RFRAME_HEAD_SIZE + RFRAME_TAIL_SIZE];
    struct
    {
        uint8_t     framehead[RFRAME_HEAD_SIZE]; //帧头
        un_floate   fdata;                      //数据。1个浮点
        uint8_t     frametail[RFRAME_TAIL_SIZE]; //帧尾
    };
} RecFrame;

typedef struct vofaCommand
{
	uint8_t        cmdType;
	uint8_t        cmdID;
    uint8_t     vofaRxBufferIndex;  //接收指针
    uint8_t     completionFlag;
	float       floatData;
	RecFrame    uartRxPacket;       //串口数据包接收数组
} vofaCommand;

#ifdef __cplusplus
extern "C" {
#endif

void vofaSendJustFloat(vofaJustFloatFrame* vofaJFFrame);     //以JustFloat协议发送数据
void vofaSendFirewater(const float* fdata, uint32_t ulSize); //以Firewater协议发送数据
void vofaSendRawdata(uint8_t* pData, uint32_t ulSize);       //以rawdata协议发送数据

void vofaSetJustFloat(vofaJustFloatFrame*, const uint8_t* , const TxChannel , const uint32_t ); //设置指定通道的JustFloat数据
void vofaJustFloatInit(void);        //Justfloat协议初始化
void uartCMDRecv(uint8_t byte_data); //uart串口接收单字节并存入vofaCommandData数据包
vofaCommand* vofaCommandParse(void);         //解析命令

extern vofaJustFloatFrame JustFloat_Data;  //包含接收到的浮点数据的结构体
extern vofaCommand        vofaCommandData; //包含命令的结构体

#ifdef __cplusplus
}
#endif

#endif
