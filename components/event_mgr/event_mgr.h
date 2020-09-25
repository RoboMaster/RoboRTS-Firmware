/**
*  @file event_mgr.c/h
*  @version 0.9.8
*  @date 2017-09-24
*  @author sky
*
*  @brief
*   2017-09-24
*   0.9: 利用事件标志组实现通知功能
*   0.9.1: 优化通知功能。
*           1，考虑到事件数量通常会远远比任务数多，所以将通知对象转移到订阅者中；减少了内存占用和代码量
*           2，将事件标志组更换为任务通知功能
*   0.9.2：考虑到创建事件时并不知道会有多少订阅者，不好估计消息缓存数量，
*           所以改成在订阅时增加内存池大小，订阅者根据自身情况给出新增的消息队列数量
*   0.9.3：修复NOLIST模式下，获取完消息后还能够再次获取的问题（每次获取消息后应该清空通知数量）
*           将订阅者模式设置从注册订阅函数更改到订阅者初始化函数，防止误用
*
*   2017-10-26
*   0.9.4：增加推送超时（利用操作系统延时）,增加时间戳选项
*
*   2017-11-02
*   0.9.5：分离两种模式，队列模式加入注册回调， 两种模式订阅者都可以订阅多条
*
*   2017-11-20
*   0.9.6：考虑到模块临界区代码都不长，所以将互斥量去除，用开关全局中断代替，经过测试，
*           在STM32F427@180MHz平台上，2Byte长度消息投递到接收的时间，从17us缩短为6us
*           （单纯内存拷贝512个字节需要3us时间）
*
*   2017-12-13
*   0.9.7：修复取消订阅最后一个订阅者时的一个0指针bug
*
*   2018-03-30
*   0.9.8：修复一个事件id可能会创建两个对象的bug
*
*  @attention
*
*
*  @copyright 2017 RoboMaster. All right reserved.
*
*/


#ifndef MSG_MGR_H
#define MSG_MGR_H


#include <stdint.h>
#include <string.h>

#include "memory_mgr.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "sys.h"

/*
使用通知功能，如果在中断中使用，必须关闭此宏
如果开启，订阅者可以阻塞等待，在事件更新时会自动唤醒挂起的订阅者（基于任务通知）
如果关闭，订阅者需要周期的查询是否有新消息到来
*/
//#define EVENT_USE_NOTIFY


/* 开关全局中断宏定义 */
#define EVENT_DISABLE_IRQ()     MASTER_INT_DISABLE()
#define EVENT_ENABLE_IRQ()      MASTER_INT_ENABLE()

/*
使用时间戳功能
开启后每条消息都会附带一个时间戳
需要定义时间戳获取宏： EVENT_GET_TIMESTAMP()
 */
#define EVENT_USE_TIMESTAMP

/* 获取时间戳宏定义 */
#ifdef EVENT_USE_TIMESTAMP
    #define EVENT_GET_TIMESTAMP()   get_time_ms()
#endif



/* 错误类型定义 */
#define EVENT_WARN_MEMORY_RETRY      1  /* 重试发送成功 */
#define EVENT_ERR_NONE               0  /* 没有错误 */
#define EVENT_ERR_MEMORY_LACK       -1  /* 内存申请失败 */
#define EVENT_ERR_EMPTY_POINT       -2  /* 空指针 */
#define EVENT_ERR_INVALIDE_EVENT    -3  /* 事件没有找到 */
#define EVENT_ERR_EVENT_UNSUBD      -4  /* 没有订阅该事件 */


/* 工作模式 */
typedef enum
{
    SUBS_MODE_NORMAL = 0,    /* 1,normal模式下，订阅者保存消息队列，只要给定的消息队列长度足够就不会漏消息，注册的时候直接把处理
                                    函数传参进去即可
                                2,normal模式下，一个订阅者可以订阅任意多个event */

    SUBS_MODE_NOLIST,        /* 1,noList模式下，订阅者中每一个eventID只申请一个内存单元，保留最新的消息，需要使用 EventMsgGetLast
                                    拷贝数据
                                2,nolist模式下，一个订阅者可以订阅任意多个event */
} subscribeMode_t;



/* 事件订阅队列头 */
typedef struct ListHead_t
{
    struct ListHead_t *next;
} listHead_t;

/* 事件数据类型 */
typedef struct Event_t
{
    struct Event_t  *next;
    uint32_t        eventID;
    uint16_t        msgSize;            /* 每个消息的大小，单位字节 */
    uint8_t         msgNum;             /* 消息个数 */
    uint8_t         subsNum;            /* 订阅者数量 */
    listHead_t      subsListHead;       /* 订阅者列表，挂 subsList_t 结构 */
    memUint_t       mmu;                /* 内存管理单元 */
    // xSemaphoreHandle mutex;             /* 互斥访问 */
} event_t;

/* 推送者数据类型 */
typedef event_t *publisher_t;

/* 事件句柄 */
typedef event_t *eventHandler_t;

/* 事件列表头 */
typedef struct
{
    event_t *next;
}
headEvent_t;

/* 事件消息数据类型 */
typedef struct EventMsg_t
{
    struct EventMsg_t   *next;
    event_t             *pEvent;
#ifdef EVENT_USE_TIMESTAMP
    uint32_t            timeStamp;
#endif
    uint8_t             msgData[];          /* 存储用户数据 */
} eventMsg_t;


/* 消息回调 */
typedef void (*msgCallBack_f)(uint32_t eventID, void *pMsgData, uint32_t timeStamp);


/* 订阅事件列表项 */
typedef struct SubsEvent_t
{
    struct SubsEvent_t  *next;
    uint32_t            eventID;
    void                *callBackOrMemory;
} subsEvent_t;

/* 订阅者数据类型 */
typedef struct Subscriber_t
{
    eventMsg_t  *pMsgHead;
    eventMsg_t  *pMsgTail;
    uint32_t    subsMode;        /* 订阅模式 subscribeMode_t */
    listHead_t  eventListHead;   /* 挂 subsEvent_t 结构 */
    // xSemaphoreHandle mutex;

#ifdef EVENT_USE_NOTIFY
    TaskHandle_t taskHandler;
#endif
} subscriber_t;

/* 订阅者列表项 */
typedef struct SubsList_t
{
    struct SubsList_t *next;
    subscriber_t      *subscriber;
} subsList_t;




int EventSubscribeInit(subscriber_t *pSubscriber, subscribeMode_t mode);
int EventSubscribe(subscriber_t *pSubscriber, uint32_t eventID, uint32_t msgSize, uint32_t msgAddNum, msgCallBack_f handlerFun);
int EventUnsubscribe(subscriber_t *pSubscriber, uint32_t eventID);

int EventPostInit(publisher_t *publisher, uint32_t eventID, uint32_t msgSize);
int EventMsgPost(publisher_t *publisher, void *pMsgData, TickType_t waitTicks);

void EventMsgProcess(subscriber_t *pSubscriber, TickType_t waitTicks);
int EventMsgGetLast(subscriber_t *pSubscriber, uint32_t eventID, void *pMsgAddr, uint32_t *pTimeStamp);


#endif


