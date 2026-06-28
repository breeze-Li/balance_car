#ifndef __FIFO_H__
#define __FIFO_H__

//#include "includes.h"
#include <stdint.h>

typedef enum
{
    FIFO_ERROR_NONE = 0,    //无错误
    FIFO_ERROR_ENTER,       //输入参数错误
    FIFO_ERROR_FULL,        //数据已存满
    FIFO_ERROR_NOSPACE,     //无空间
    FIFO_ERROR_DEL_EMPTY,   //无数据可删除
} fifo_error_t;

typedef struct
{
    uint16_t bufflen;       //Buff已有长度
    uint16_t maxLen;        //数据最大缓存长度
    uint16_t front;         //头
    uint16_t rear;          //尾
    uint8_t *dataBuff;      //Buff地址
} fifo_t;

typedef struct list_node
{
    uint16_t priority;      //数据优先级
    uint16_t length;        //当前数据长度
    struct list_node *pre;  //头指针
    struct list_node *next; //尾指针
} list_t;


/*******************************************
//              fifo使用步骤
//1.新建一个fifo_t对像
//2.使用fifoInit()函数初始化fifo
//3.使用fifoEnqueue()入列不定长度数据或者使用fifoEnqueueEx()入列单个数据
//4.使用fifoDequeue()出列不定长度数据或者使用fifoDequeueEx()出列单个数据
//5.可以使用fifoPeak()函数判断当前fifo中是否有数据或者查看有多少数据
*******************************************/


/*******************************************
//fifo初始化
//参数:*fifo, fifo对像
//参数:*buffer, 存储数据的指针.此批针为真正数据存储的位置
//参数:size, 存储数据空间的大小.
//说明:使用fifo前,必须要用这个函数初始化
*******************************************/
void fifoInit(fifo_t *fifo, uint8_t *buffer, uint16_t size);

/*******************************************
//fifo入列
//参数:*fifo, fifo对像
//参数:*buffer, 需要入列数据的指针.
//参数:len, 需要入列数据的长度.
//说明:如果入列数据大于剩余空间的大小,则超出空间的数据被抛弃
*******************************************/
uint16_t fifoEnqueue(fifo_t *fifo, uint8_t *buffer, uint16_t len);

/*******************************************
//fifo出列
//输入
//参数:*fifo, fifo对像
//参数:*buffer, 接收出列数据的指针.
//参数:maxLen, 接收出列数据的最大长度.
//输出
//uint16_t: 实际出列数据的长度
//说明:一次出列最多为maxLen长度
*******************************************/
uint16_t fifoDequeue(fifo_t *fifo, uint8_t *buffer, uint16_t maxLen);

/*******************************************
//fifo单字节入列
//参数:*fifo, fifo对像
//参数:data, 需要入列的数据.
//说明:无
*******************************************/
void fifoEnqueueEx(fifo_t *fifo, uint8_t data);

/*******************************************
//fifo单字节出列
//输入
//参数:*fifo, fifo对像
//输出
//uint8_t: 出列的字节数据
//说明:无
*******************************************/
uint8_t fifoDequeueEx(fifo_t *fifo);

/*******************************************
//查看当前fifo中已有多少个数据
//输入
//参数:*fifo, fifo对像
//输出
//uint16_t: 已有多少个数据
//说明:无
*******************************************/
uint16_t fifoPeak(fifo_t *fifo);




/*******************************************
//              list使用步骤
//1.新建一个list_t对像
//2.listInit()函数初始化list
//3.使用listEnqueue()入列不定长度数据,算为一个节点
//4.使用listDequeue()出列一个节点的数据
//5.可以使用listPeak()函数查看最近一个节点有多少数据
//6.可以使用listGetCount()查看有list中总共有多少个节点
//7.在重新使用listInit前,需要清空里面所有数据,防止内存泄露
*******************************************/


/*******************************************
//列表初始化
//参数:*list, list对像
//参数:maxNodeCounter, 最大节点数,意指此列表中,最大可存多少组数据.
//说明:1.使用fifo前,必须要用这个函数初始化.
//     2.必须做最大节点数限制,如不加限制且使用不当的话,可能会把动态内存消耗完.
*******************************************/
void listInit(list_t *list, uint16_t maxNodeCounter);

/*******************************************
//入列
//输入
//参数:*list, list对像
//参数:*pdata, 需要存入的数据指针
//参数:len, 需要存入的数据长度
//参数:priority, 优先级. 数字越小,出列的优先级越高.
//输出
//fifo_error_t: 最返回执行状态. 正常:FIFO_ERROR_NONE
//说明:1.函数会判断当前总节点数是否<最大节点数则执行入列,否则数据抛弃. 返回 FIFO_ERROR_FULL
//     2.函数会申请动态内存,如申请失败,会返回 FIFO_ERROR_NOSPACE
//     3.如在中断中调用,则需关总中断
*******************************************/
fifo_error_t listEnqueue(list_t *list, uint8_t *pdata, uint16_t len, uint8_t priority);

/*******************************************
//出列
//输入
//参数:*list, list对像
//参数:*pdata, 需要接收出列数据的指针
//参数:maxLen, 需要接收出列数据的最大长度
//输出
//uint16_t: 最终出列的实际数据长度.如果返回长度为0,表示list中没有数据可输出
//说明:1.数据出列后,会在list中删除已出列的数据,并释放对应内存
//     2.如在中断中调用,则需关总中断
*******************************************/
uint16_t listDequeue(list_t *list, uint8_t *pdata, uint16_t maxLen);

/*******************************************
//查看对应序号中的数据
//输入
//参数:*list, list对像
//参数:**pdata, 查看数据的多重指针,查到数据后此指针会指向动态内存中的数据地址,不会产生复制数据的动作.
//参数:index, 需要查看第几个数据
//输出
//uint16_t: 所查看的数据长度
//说明:1.不会删除数据. 在某些用法上有妙用.
//     2.如在中断中调用,则需关总中断
*******************************************/
uint16_t listGetPtr(list_t *list, uint8_t **pdata, uint16_t index);

/*******************************************
//删除指定序号数据
//输入
//参数:*list, list对像
//参数:index, 需要删除第几个数据
//输出
//uint16_t: 返回操作状态正常:FIFO_ERROR_NONE 失败:FIFO_ERROR_DEL_EMPTY
//说明:1.数据删除后并释放对应内存
//     2.如在中断中调用,则需关总中断
*******************************************/
fifo_error_t listDelete(list_t *list, uint16_t index);

/*******************************************
//查看最近一个节点的数据长度
//输入
//参数:*list, list对像
//输出
//uint16_t: 返回最近一个节点的数据长度,如果没有节点,则返回当前节点个数,即0
//说明:无
*******************************************/
uint16_t listPeak(list_t *list);

/*******************************************
//返回节点个数
//输入
//参数:*list, list对像
//输出
//uint16_t: 返回节点个数
//说明:无
*******************************************/
uint16_t listGetCount(list_t *list);


#endif
