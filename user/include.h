#ifndef INCLUDE_H
#define INCLUDE_H

#include <stdint.h>
#include "module.h"
#include "delay.h"
#include "app_usart2.h"

// 联合体定义：将一个 uint16_t 拆分为两个 uint8_t
typedef union {
    uint16_t hword;           // 16位整数值
    struct {
        uint8_t L;         // 低8位
        uint8_t H;        // 高8位
    };
} un16_t;

typedef void (*FCT_VOID)(void);
typedef void (*FCT_UINT8)(uint8_t);
typedef uint8_t (*UINT8_FCT)(void);
typedef int16_t (*INT16_FCT)(void);



#ifdef  FALSE
#undef  FALSE
#endif
#define FALSE   (0)

#ifdef  TRUE
#undef  TRUE
#endif
#define TRUE    (1)

#ifdef  NULL
#undef  NULL
#endif
#define NULL    (0)

#ifdef  ON
#undef  ON
#endif
#define ON      (1)

#ifdef  OFF
#undef  OFF
#endif
#define OFF     (0)

#ifdef  HIGH
#undef  HIGH
#endif
#define HIGH    (1)

#ifdef  LOW
#undef  LOW
#endif
#define LOW     (0)

#ifndef SKIP
#define SKIP    (2)
#endif

#endif
