#ifndef INCLUDE_H
#define INCLUDE_H

#include <stdint.h>
#include "module.h"
#include "delay.h"
#include "app_usart2.h"
#include "vofa_uart.h"

typedef uint32_t    timer_t;
typedef union
{
    uint8_t  byte[2];
    uint16_t hword;
} un16_t;

typedef union
{
    uint8_t  byte[4];
    uint16_t hword[2];
    uint32_t word;
} un32_t;

typedef union
{
    uint8_t  byte[8];
    uint16_t hword[4];
    uint32_t word[2];
    uint64_t dword;
} un64_t;

// 联合体定义：将一个 uint16_t 拆分为两个 uint8_t
typedef union {
    uint16_t hword;           // 16位整数值
    struct {
        uint8_t L;         // 低8位
        uint8_t H;        // 高8位
    };
} un16_hl_t;

//#define RAM_FUNC                    __attribute__((section("CCMRAM")))

#define TICK_SECOND                 1000u
#define TICK_GET                    GetTick
#define TICK_RESET(x)               ((x) = GetTick())
#define TICK_COUNTER(x, y)          ((GetTick() - (x)) >= (y))
#define DELAY_MS(x)                 Delay(x)

#define S8_MAX                      ((int8_t)127)
#define S8_MIN                      ((int8_t)-128)
#define U8_MAX                      ((uint8_t)255u)
#define S16_MAX                     ((int16_t)32767)
#define S16_MIN                     ((int16_t)-32768)
#define U16_MAX                     ((uint16_t)65535u)
#define S32_MAX                     ((int32_t)2147483647)
#define S32_MIN                     ((int32_t)-2147483648)
#define U32_MAX                     ((uint32_t)4294967295uL)

#ifndef BITSET
#define BITSET(X, MASK)             ((X) |= (MASK))
#endif

#ifndef BITCLR
#define BITCLR(X, MASK)             ((X) &= ~(MASK))
#endif

#ifndef BITGET
#define BITGET(X, MASK)             (((X) & (MASK)) ? 1 : 0)
#endif

#define dim(x)                      (sizeof(x) / sizeof(x[0]))                  /* Returns number of elements */
#define MAX(a, b)                   ((a) > (b) ? (a) : (b))
#define MIN(a, b)                   ((a) < (b) ? (a) : (b))
#define MID(a, b)                   (((a) + (b)) / 2)
#define SWAP8(x)                    (((x) << 4 ) | ((x) >> 4 ))
#define SWAP16(x)                   (((x) << 8 ) | ((x) >> 8 ))
#define SWAP32(x)                   (((x) << 16) | ((x) >> 16))
#define SWAP_BYTES(x)               __swap_bytes(x)

#define RANGE_SET(x, min, max, set) ((x) = ((x) >= (min) && (x) <= (max)) ? (x) : (set))    //x:源值, min:最小范围值, max:最大范围值, 不在范围内则设为给出的设定值
#define RANGE_CHECK(x, min, max)    (((x) >= (min) && (x) <= (max)))            //x:源值, min:最小范围值, max:最大范围值
#define OFFSET_CHECK(a, b, offset)  (abs((int32_t)(a)-(int32_t)(b)) <= (offset))//a与b的差值在x范围内
#define ADD_LOOP(add, min, max)     {if(add < (max)) {add++;} else {add = min;}}
#define DEC_LOOP(dec, min, max)     {if(dec > min) {dec--;} else {dec = (max);}}
#define ADD_2_MAX(add, max)         {if(add < (max)) {add++;} else {add = max;}}
#define DEC_2_MIN(dec, min)         {if(dec > min) {dec--;} else {dec = (min);}}

#define REG_READ8(reg)              (*((uint8_t *)(reg)))
#define REG_READ16(reg)             (*((uint16_t *)(reg)))
#define REG_READ32(reg)             (*((uint32_t *)(reg)))
#define REG_WRITE8(reg, data)       (*((uint8_t *)(reg)) = (data))
#define REG_WRITE16(reg, data)      (*((uint16_t *)(reg)) = (data))
#define REG_WRITE32(reg, data)      (*((uint32_t *)(reg)) = (data))


#define ENTER_CRITICAL()            __disable_interrupt()
#define EXIT_CRITICAL()             __enable_interrupt()
#define DINT()                      __disable_interrupt()
#define EINT()                      __enable_interrupt()
#define NOP()                       __no_operation()

typedef void (*FCT_VOID)(void);
typedef void (*FCT_UINT8)(uint8_t);
typedef uint8_t (*UINT8_FCT)(void);
typedef int16_t (*INT16_FCT)(void);

#ifndef BIT0
#define BIT0                        (0x0001)
#define BIT1                        (0x0002)
#define BIT2                        (0x0004)
#define BIT3                        (0x0008)
#define BIT4                        (0x0010)
#define BIT5                        (0x0020)
#define BIT6                        (0x0040)
#define BIT7                        (0x0080)
#define BIT8                        (0x0100)
#define BIT9                        (0x0200)
#define BIT10                       (0x0400)
#define BIT11                       (0x0800)
#define BIT12                       (0x1000)
#define BIT13                       (0x2000)
#define BIT14                       (0x4000)
#define BIT15                       (0x8000)
#endif

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
