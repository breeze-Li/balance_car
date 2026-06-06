/******************************************************************************
 * @brief    系统模块管理(包含系统初始化,时间片轮询系统)
 *
 * Copyright (c) 2017~2020, <morro_luo@163.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs: 
 * Date           Author       Notes 
 * 2016-06-24     Morro        初版完成
 * 2020-05-23     Morro        增加匿名类型,防止模块重名错误
 * 2020-06-28     Morro        增加is_timeout超时判断接口
 ******************************************************************************/

#ifndef _MODULE_H_
#define _MODULE_H_

#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

/*匿名类型定义 -----------------------------------------------------------*/
#define  ANONY_CONN(type, var, line)  type  var//##line
#define  ANONY_DEF(type,prefix,line)  ANONY_CONN(type, prefix, line)
#define  ANONY_TYPE(type,prefix)      ANONY_DEF(type, prefix, __LINE__)

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ( \
	(type *)( (char *)(ptr) - offsetof(type,member) ))


#if defined(__CC_ARM) || defined(__GNUC__) /* ARM,GCC*/
    #define SECTION(x)                  __attribute__((section(x)))
    #define UNUSED                      __attribute__((unused))
    #define USED                        __attribute__((used))
    #define ALIGN(n)                    __attribute__((aligned(n)))
    #define WEAK                        __attribute__((weak))
#elif defined (__ICCARM__)              /*IAR */
    #define SECTION(x)                  @ x
    #define UNUSED                      
    #define USED                        __root
    #define WEAK                        __weak
#else
    #error "Current tool chain haven't supported yet!"
#endif

#ifdef __cplusplus
}
#endif

/*模块初始化项*/
typedef struct {
    const char *name;               //模块名称
    void (*init)(void);             //初始化接口
}init_item_t;

/*任务处理项*/
typedef struct {
    const char *name;               //模块名称    
    void (*handle)(void);           //初始化接口
    unsigned int interval;          //轮询间隔
    unsigned int *timer;            //指向定时器指针
}task_item_t;

#define __module_initialize(name,func,level)           \
    USED ANONY_TYPE(const init_item_t, init_tbl_##func)\
    SECTION("init.item."level) = {name,func}

/*
 * @brief       任务注册
 * @param[in]   name    - 任务名称 
 * @param[in]   handle  - 初始化处理(void func(void){...})
  * @param[in]  interval- 轮询间隔(ms)
 */
#define task_register(name, handle,interval)                \
    static unsigned int __task_timer_##handle;              \
    USED ANONY_TYPE(const task_item_t, task_item_##handle)  \
    SECTION("task.item.1") =                                \
    {name,handle, interval, &__task_timer_##handle}

/*
 * @brief       模块初始化注册
 * @param[in]   name    - 模块名称 
 * @param[in]   func    - 初始化入口函数(void func(void){...})
 */
#define system_init(name,func)  __module_initialize(name,func,"1")
#define driver_init(name,func)  __module_initialize(name,func,"2")
#define module_init(name,func)  __module_initialize(name,func,"3")

void systick_increase(unsigned int ms);
unsigned int get_tick(void);
bool is_timeout(unsigned int start, unsigned int timeout);
void module_task_init(void);
void module_task_process(void);
    
#endif
