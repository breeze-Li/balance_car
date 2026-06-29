#ifndef APP_USART2_H
#define APP_USART2_H

#include "stm32f10x.h"
#include "usart.h"

void App_USART3_Init(void);
uint16_t comHwSend(uint8_t *buffer, uint16_t size);
uint16_t comHwRead(uint8_t *buffer, uint16_t size);

#endif
