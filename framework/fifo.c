#include "fifo.h"
#include <string.h>  // 假设包含 memcpy 函数

/* FIFO 初始化 */
void fifoInit(fifo_t *fifo, uint8_t *buffer, uint16_t size) {
    fifo->dataBuff = buffer;
    fifo->maxLen = size;
    fifo->front = 0;
    fifo->rear = 0;
    fifo->bufflen = 0;
}

/* 批量入队（超出fifo剩余长度的数据被部分丢弃） */
uint16_t fifoEnqueue(fifo_t *fifo, uint8_t *buffer, uint16_t len) {
    uint16_t available = fifo->maxLen - fifo->bufflen;
    uint16_t write_len = (len > available) ? available : len;

    if (write_len == 0) return 0;

    // 分两段写入：从 rear 到缓冲区末尾，剩余部分从缓冲区开头开始
    uint16_t first_part = fifo->maxLen - fifo->rear;
    if (first_part >= write_len) {
        memcpy(&fifo->dataBuff[fifo->rear], buffer, write_len);
        fifo->rear += write_len;
    } else {
        memcpy(&fifo->dataBuff[fifo->rear], buffer, first_part);
        memcpy(fifo->dataBuff, buffer + first_part, write_len - first_part);
        fifo->rear = write_len - first_part;
    }

    fifo->bufflen += write_len;
    if (fifo->rear >= fifo->maxLen) fifo->rear = 0;
    return write_len;
}

/* 批量出队 */
uint16_t fifoDequeue(fifo_t *fifo, uint8_t *buffer, uint16_t maxLen) {
    uint16_t read_len = (fifo->bufflen > maxLen) ? maxLen : fifo->bufflen;

    if (read_len == 0) return 0;

    // 分两段读取：从 front 到缓冲区末尾，剩余部分从缓冲区开头开始
    uint16_t first_part = fifo->maxLen - fifo->front;
    if (first_part >= read_len) {
        memcpy(buffer, &fifo->dataBuff[fifo->front], read_len);
        fifo->front += read_len;
    } else {
        memcpy(buffer, &fifo->dataBuff[fifo->front], first_part);
        memcpy(buffer + first_part, fifo->dataBuff, read_len - first_part);
        fifo->front = read_len - first_part;
    }

    fifo->bufflen -= read_len;
    if (fifo->front >= fifo->maxLen) fifo->front = 0;
    return read_len;
}

/* 单字节入队 */
void fifoEnqueueEx(fifo_t *fifo, uint8_t data) {
    if (fifo->bufflen >= fifo->maxLen) return;

    fifo->dataBuff[fifo->rear] = data;
    fifo->rear = (fifo->rear + 1) % fifo->maxLen;
    fifo->bufflen++;
}

/* 单字节出队 */
uint8_t fifoDequeueEx(fifo_t *fifo) {
    if (fifo->bufflen == 0) return 0;

    uint8_t data = fifo->dataBuff[fifo->front];
    fifo->front = (fifo->front + 1) % fifo->maxLen;
    fifo->bufflen--;
    return data;
}

/* 获取当前数据量 */
uint16_t fifoPeak(fifo_t *fifo) {
    return fifo->bufflen;
}
