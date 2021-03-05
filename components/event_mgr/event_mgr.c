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

#include "event_mgr.h"

/* 第一个event，用于注册时查找 */
static headEvent_t headEvent;


/* 定义订阅者之后，必须进行初始化才能使用 */
int EventSubscribeInit(subscriber_t *pSubscriber, subscribeMode_t mode)
{
    pSubscriber->pMsgHead = NULL;
    pSubscriber->pMsgTail = NULL;
    pSubscriber->subsMode = mode;
    pSubscriber->eventListHead.next = NULL;
    // pSubscriber->mutex = xSemaphoreCreateMutex();
    // if(pSubscriber->mutex == NULL)
    // {
    // return EVENT_ERR_MEMORY_LACK;
    // }
#ifdef EVENT_USE_NOTIFY
    pSubscriber->taskHandler = xTaskGetCurrentTaskHandle();
#endif
    return EVENT_ERR_NONE;
}


/* 正常模式下，释放消息，用户使用完队列消息之后需要释放消息内存 */
static int EventMsgFree(eventMsg_t *pMsg)
{
    if (pMsg == NULL)
    {
        return EVENT_ERR_EMPTY_POINT;
    }
    else
    {
        // xSemaphoreHandle mutex = pMsg->pEvent->mutex;
        // xSemaphoreTake(mutex, portMAX_DELAY);
        EVENT_DISABLE_IRQ();
        MemPutBlk(&(pMsg->pEvent->mmu), (void *)pMsg);
        EVENT_ENABLE_IRQ();
        // xSemaphoreGive(mutex);

        return EVENT_ERR_NONE;
    }

}



/* 内部调用，通过event id找到相应event */
static event_t *GetEventFromID(uint32_t eventID)
{
    event_t *res = NULL;
    eventHandler_t curEvent;

    for (curEvent = headEvent.next; curEvent != NULL; curEvent = curEvent->next)
    {
        if (curEvent->eventID == eventID)
        {
            res = curEvent;
            break;
        }
    }
    return res;
}

/* 内部调用，通过id寻找回调函数 */
static void *GetHandlerFromID(subscriber_t *pSubscriber, uint32_t eventID)
{
    subsEvent_t *subsEvent;

    for (subsEvent = (subsEvent_t *)&pSubscriber->eventListHead; subsEvent->next != NULL; subsEvent = subsEvent->next)
    {
        if (subsEvent->next->eventID == eventID)
        {
            return subsEvent->next->callBackOrMemory;
        }
    }
    return NULL;
}



/* 创建事件 */
static int EventCreat(eventHandler_t *pOutHandler, uint32_t eventID, uint16_t msgSize)
{
    eventHandler_t curEvent;
    uint32_t eventMsgSize = sizeof(eventMsg_t) + msgSize;
    eventHandler_t eventHandler;

    if (pOutHandler == NULL)
    {
        return EVENT_ERR_EMPTY_POINT;
    }

    taskENTER_CRITICAL();

    eventHandler = GetEventFromID(eventID);
    if (eventHandler == NULL)
    {
        /* 申请事件结构内存 */
        eventHandler = (eventHandler_t)pvPortMalloc(sizeof(event_t));
        if (eventHandler == NULL)
        {
            return EVENT_ERR_MEMORY_LACK;
        }

        /* 创建互斥量 */
        // eventHandler->mutex = xSemaphoreCreateMutex();
        // if(eventHandler->mutex == NULL)
        // {
        // vPortFree(eventHandler);
        // return EVENT_ERR_MEMORY_LACK;
        // }

        if (pOutHandler != NULL)
        {
            *pOutHandler = eventHandler;
        }

        MemPoolInit(&eventHandler->mmu, NULL, 0, eventMsgSize);

        eventHandler->eventID = eventID;
        eventHandler->msgNum = 0;
        eventHandler->msgSize = msgSize;
        eventHandler->subsListHead.next = NULL;
        eventHandler->next = NULL;

        /* 将新事件插入链表尾部 */
        for (curEvent = (event_t *)&headEvent; curEvent->next != NULL; curEvent = curEvent->next)
        {
            // __nop();
        }
        curEvent->next = eventHandler;
    }
    else
    {
        *pOutHandler = eventHandler;
    }

    taskEXIT_CRITICAL();

    return EVENT_ERR_NONE;
}



/* 注册事件，将输入的订阅对象连接到事件订阅列表中 */
/* msgAddNum：每次订阅事件时，用户需要给出一个消息数量，表示当前订阅者可能会阻塞的消息个数，该数量会转换成相应大小内存挂接到event结构中 */
/* 订阅者释放时为了安全，不会释放自己的内存池，所以如果反复的订阅/取消订阅某个事件，则只需要在第一次订阅时输入消息队列增加个数msgAddNum即可 */
int EventSubscribe(subscriber_t *pSubscriber, uint32_t eventID, uint32_t msgSize, uint32_t msgAddNum, msgCallBack_f handlerFun)
{
    int res = EVENT_ERR_NONE;
    eventHandler_t targetEvent;
    subsList_t *subsList;
    subsList_t *subsListNewItem;
    subsEvent_t *subsEvent;
    subsEvent_t *subsEventNewItem;
    // xSemaphoreHandle mutex;
    uint32_t eventMsgSize = sizeof(eventMsg_t) + msgSize;

    targetEvent = GetEventFromID(eventID);
    if (targetEvent == NULL)
    {
        EventCreat(&targetEvent, eventID, msgSize);
        if (targetEvent == NULL)
        {
            return EVENT_ERR_MEMORY_LACK;
        }
    }

    if (targetEvent != NULL)
    {
        void *memoryPool = NULL;
        // mutex = targetEvent->mutex;

        /* 申请订阅列表项空间 */
        subsListNewItem = (subsList_t *)pvPortMalloc(sizeof(subsList_t));
        if (subsListNewItem == NULL)
        {
            return EVENT_ERR_MEMORY_LACK;
        }
        else
        {
            subsListNewItem->next = NULL;
            subsListNewItem->subscriber = pSubscriber;
        }

        /* 申请订阅命令信息 */
        subsEventNewItem = (subsEvent_t *)pvPortMalloc(sizeof(subsEvent_t));
        if (subsEventNewItem == NULL)
        {
            return EVENT_ERR_MEMORY_LACK;
        }
        subsEventNewItem->next = NULL;
        subsEventNewItem->eventID = eventID;
        if (pSubscriber->subsMode == SUBS_MODE_NORMAL)
        {
            subsEventNewItem->callBackOrMemory = handlerFun;
            if (msgAddNum)
            {
                /* 申请消息内存池并初始化 */
                memoryPool = (void *)pvPortMalloc(msgAddNum * eventMsgSize);
                if (memoryPool == NULL)
                {
                    return EVENT_ERR_MEMORY_LACK;
                }
            }

            /* 将新申请的订阅命令信息挂入订阅命令队列尾部 */
            for (subsEvent = (subsEvent_t *)&pSubscriber->eventListHead; subsEvent->next != NULL; subsEvent = subsEvent->next)
            {
                // nop
            }
            subsEvent->next = subsEventNewItem;

            // xSemaphoreTake(mutex, portMAX_DELAY);
            EVENT_DISABLE_IRQ();
            if (msgAddNum)
            {
                /* 将新申请的内存挂到订阅的event内存管理单元mmu当中 */
                MemAddBlksToPool(&targetEvent->mmu, memoryPool, msgAddNum, eventMsgSize);
            }
            /* 将新列表项加入到事件订阅列表中 */
            for (subsList = (subsList_t *)&targetEvent->subsListHead; subsList->next != NULL; subsList = subsList->next)
            {
                // nop
            }
            subsList->next = subsListNewItem;
            targetEvent->subsNum ++;
            EVENT_ENABLE_IRQ();
            // xSemaphoreGive(mutex);
        }
        else if (pSubscriber->subsMode == SUBS_MODE_NOLIST)
        {
            /* 申请消息内存，存入对应订阅eventid项 */
            memoryPool = (void *)pvPortMalloc(1 * eventMsgSize);
            subsEventNewItem->callBackOrMemory = memoryPool;
            if (memoryPool == NULL)
            {
                return EVENT_ERR_MEMORY_LACK;
            }

            /* 将新申请的订阅命令信息挂入订阅命令队列尾部 */
            for (subsEvent = (subsEvent_t *)&pSubscriber->eventListHead; subsEvent->next != NULL; subsEvent = subsEvent->next)
            {
                // nop
            }
            subsEvent->next = subsEventNewItem;

            // xSemaphoreTake(mutex, portMAX_DELAY);
            EVENT_DISABLE_IRQ();
            /* 将新列表项加入到事件订阅列表中 */
            for (subsList = (subsList_t *)&targetEvent->subsListHead; subsList->next != NULL; subsList = subsList->next)
            {
                // nop
            }
            subsList->next = subsListNewItem;
            targetEvent->subsNum ++;
            EVENT_ENABLE_IRQ();
            // xSemaphoreGive(mutex);
        }
    }

    return res;
}


/* 尽量避免频繁的取消订阅事件*/
/* 订阅者释放时为了安全，不会释放自己的内存池，所以如果反复的订阅/取消订阅某个事件，则只需要在第一次订阅时输入消息队列增加个数msgAddNum即可 再次订阅msgAddNum填0*/
int EventUnsubscribe(subscriber_t *pSubscriber, uint32_t eventID)
{
    int res;
    eventHandler_t targetEvent;
    subsList_t *subs;
    subsEvent_t *subsEvent;
    // xSemaphoreHandle mutex;

    targetEvent = GetEventFromID(eventID);
    if (targetEvent == NULL)
    {
        res = EVENT_ERR_INVALIDE_EVENT;
    }
    else
    {
        /* 释放订阅命令列表项 */
        res = EVENT_ERR_EVENT_UNSUBD;
        for (subsEvent = (subsEvent_t *)&pSubscriber->eventListHead; subsEvent->next != NULL; subsEvent = subsEvent->next)
        {
            if (subsEvent->next->eventID == eventID)
            {
                if (pSubscriber->subsMode == SUBS_MODE_NOLIST)
                {
                    vPortFree(subsEvent->next->callBackOrMemory);
                }
                vPortFree(subsEvent->next);
                subsEvent->next = subsEvent->next->next;
                break;
            }
        }

        /* 释放事件订阅列表项 */
        // mutex = targetEvent->mutex;
        // xSemaphoreTake(mutex, portMAX_DELAY);
        EVENT_DISABLE_IRQ();
        for (subs = (subsList_t *)&targetEvent->subsListHead; subs->next != NULL; subs = subs->next)
        {
            if (subs->next->subscriber == pSubscriber)
            {
                vPortFree(subs->next);
                subs->next = subs->next->next;
                targetEvent->subsNum --;
                res = EVENT_ERR_NONE;
                break;
            }
        }
        EVENT_ENABLE_IRQ();
        // xSemaphoreGive(mutex);
    }

    return res;
}

/* 注册成为发送者，实际上是得到事件句柄，如果已知事件句柄，则可以直接推送消息 */
int EventPostInit(publisher_t *publisher, uint32_t eventID, uint32_t msgSize)
{
    int res = EVENT_ERR_NONE;
    eventHandler_t targetEvent;

    targetEvent = GetEventFromID(eventID);
    if (targetEvent == NULL)
    {
        res = EventCreat(&targetEvent, eventID, msgSize);
        if (res == EVENT_ERR_NONE)
        {
            *publisher = (publisher_t)targetEvent;
        }
    }
    else
    {
        *publisher = targetEvent;
    }

    return res;
}


/* 发送消息 */
int EventMsgPost(publisher_t *publisher, void *pMsgData, TickType_t waitTicks)
{
    int res = EVENT_ERR_NONE;
    uint16_t size = (*publisher)->msgSize;
    eventMsg_t *pEventMsg;
    subsList_t *pSubsList;
    // xSemaphoreHandle mutex;
#ifdef EVENT_USE_TIMESTAMP
    uint32_t timeStamp = EVENT_GET_TIMESTAMP();
#endif

    pSubsList = (subsList_t *)(*publisher)->subsListHead.next;
    if ((*publisher == NULL) || (pSubsList == NULL))
    {
        res = EVENT_ERR_EMPTY_POINT;
    }
    else
    {
        for (; pSubsList != NULL; pSubsList = pSubsList->next)
        {
            subscriber_t *curSubs = pSubsList->subscriber;

            /* 队列模式下，申请新内存并挂到队列尾部 */
            if (curSubs->subsMode == SUBS_MODE_NORMAL)
            {
                /* 申请内存存储消息 */
                // mutex = (*publisher)->mutex;
                // xSemaphoreTake(mutex, portMAX_DELAY);
                EVENT_DISABLE_IRQ();
                pEventMsg = MemGetBlk(&(*publisher)->mmu);
                EVENT_ENABLE_IRQ();
                // xSemaphoreGive(mutex);
                if (pEventMsg == NULL)
                {
                    res = EVENT_WARN_MEMORY_RETRY;
                }

                /* 如果申请内存失败，则重试 */
                /* 设置延时1个tick时，实际上绝大部分时候都会不到1个tick，所以特殊处理 */
                if (waitTicks == 1)
                {
                    waitTicks = 2;
                }
                while ((pEventMsg == NULL) && (waitTicks > 0))
                {
                    waitTicks --;
                    // xSemaphoreTake(mutex, portMAX_DELAY);
                    EVENT_DISABLE_IRQ();
                    pEventMsg = MemGetBlk(&(*publisher)->mmu);
                    EVENT_ENABLE_IRQ();
                    // xSemaphoreGive(mutex);
                    return -1;
                    //vTaskDelay(1);
                }

                /* 填充新消息 */
                if (pEventMsg != NULL)
                {
                    memcpy(pEventMsg->msgData, pMsgData, size);
                    pEventMsg->pEvent = (*publisher);
                    pEventMsg->next = NULL;
#ifdef EVENT_USE_TIMESTAMP
                    pEventMsg->timeStamp = timeStamp;
#endif

                    // mutex = curSubs->mutex;
                    // xSemaphoreTake(mutex, portMAX_DELAY);
                    /* 将新消息投递到订阅者消息列表的尾部 */
                    EVENT_DISABLE_IRQ();
                    if (curSubs->pMsgHead == NULL)
                    {
                        curSubs->pMsgHead = pEventMsg;
                        curSubs->pMsgTail = pEventMsg;
                    }
                    else
                    {
                        curSubs->pMsgTail->next = pEventMsg;
                        curSubs->pMsgTail = pEventMsg;
                    }
                    EVENT_ENABLE_IRQ();
                    // xSemaphoreGive(mutex);
                }
                else
                {
                    res = EVENT_ERR_MEMORY_LACK;
                }

#ifdef EVENT_USE_NOTIFY
                xTaskNotifyGive(curSubs->taskHandler);
#endif
            }
            /* 非队列模式下，直接更新消息内存 */
            else
            {
                pEventMsg = GetHandlerFromID(curSubs, (*publisher)->eventID);
                if (pEventMsg != NULL)
                {
                    // mutex = curSubs->mutex;
                    // xSemaphoreTake(mutex, portMAX_DELAY);
                    EVENT_DISABLE_IRQ();
                    memcpy(pEventMsg->msgData, pMsgData, size);
                    pEventMsg->pEvent = (*publisher);
#ifdef EVENT_USE_TIMESTAMP
                    pEventMsg->timeStamp = timeStamp;
#else
                    pEventMsg->timeStamp = 0;
#endif
                    EVENT_ENABLE_IRQ();
                    // xSemaphoreGive(mutex);
                }
                else
                {
                    res = EVENT_ERR_EVENT_UNSUBD;
                }
            }
        }
    }
    return res;
}



/* 获取消息，如果队列中没有消息则返回NULL */
static eventMsg_t *EventMsgPull(subscriber_t *pSubscriber, TickType_t waitTicks)
{
    eventMsg_t *pMsg = NULL;
    eventMsg_t *pHeadMsg;
    uint32_t notifyValue;
    // xSemaphoreHandle mutex = pSubscriber->mutex;
#ifdef EVENT_USE_NOTIFY
    notifyValue = ulTaskNotifyTake(pdFALSE, waitTicks);
#else
    (void)waitTicks;
    notifyValue = 1;    /* 如果不使用通知功能，则设置为1，后续查看列表是否为空 */
#endif

    if (notifyValue > 0)
    {
        // xSemaphoreTake(mutex, portMAX_DELAY);
        EVENT_DISABLE_IRQ();
        pHeadMsg = pSubscriber->pMsgHead;
        if (pHeadMsg != NULL)
        {
            pMsg = pHeadMsg;
            pHeadMsg = pHeadMsg->next;    /* 消息已读取，指针后移 */
        }
        pSubscriber->pMsgHead = pHeadMsg;
        EVENT_ENABLE_IRQ();
        // xSemaphoreGive(mutex);
    }

    return pMsg;
}


/* 处理消息，自动调用回调，设置的等待时间是每次等待的时间，实际可能更长 */
void EventMsgProcess(subscriber_t *pSubscriber, TickType_t waitTicks)
{
    uint32_t msgId;
    eventMsg_t *pMsg;
    msgCallBack_f handlerFun;

    do
    {
        pMsg = EventMsgPull(pSubscriber, waitTicks);
        if (pMsg != NULL)
        {
            msgId = pMsg->pEvent->eventID;

            handlerFun = (msgCallBack_f)GetHandlerFromID(pSubscriber, msgId);
            if (handlerFun != NULL)
            {
#ifdef EVENT_USE_TIMESTAMP
                handlerFun(msgId, pMsg->msgData, pMsg->timeStamp);
#else
                handlerFun(msgId, pMsg->msgData, 0);
#endif
            }

            EventMsgFree(pMsg);
        }
    }
    while (pMsg != NULL);
}


/* 获取最新消息，只用于非队列模式 */
int EventMsgGetLast(subscriber_t *pSubscriber, uint32_t eventID, void *pMsgAddr, uint32_t *pTimeStamp)
{
    eventMsg_t *pMsg;
    // xSemaphoreHandle mutex;

    if (pMsgAddr == NULL)
    {
        return EVENT_ERR_EMPTY_POINT;
    }
    pMsg = GetHandlerFromID(pSubscriber, eventID);
    if (pMsg == NULL)
    {
        return EVENT_ERR_EVENT_UNSUBD;
    }
    if (pMsg->pEvent == NULL)
    {
        return EVENT_ERR_INVALIDE_EVENT;
    }

    // mutex = pSubscriber->mutex;
    // xSemaphoreTake(mutex, portMAX_DELAY);
    EVENT_DISABLE_IRQ();
    memcpy(pMsgAddr, pMsg->msgData, pMsg->pEvent->msgSize);
    if (pTimeStamp != NULL)
    {
#ifdef EVENT_USE_TIMESTAMP
        *pTimeStamp = pMsg->timeStamp;
#else
        *pTimeStamp = 0;
#endif
    }
    EVENT_ENABLE_IRQ();
    // xSemaphoreGive(mutex);
    return EVENT_ERR_NONE;
}



