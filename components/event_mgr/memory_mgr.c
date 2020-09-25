/**
*  @file memory_mgr.c/h
*  @version 0.9.1
*  @date 2017-09-12
*  @author sky
*
*  @brief
*   0.9: 基本功能，固定大小的内存块复用
*   0.9.1：新增内存池扩展功能
*
*  @attention
*   没有做线程安全，使用时必须加锁
*
*
*  @copyright 2017 RoboMaster. All right reserved.
*
*/

#include "memory_mgr.h"


/* 将新的内存块加入内存池 */
void MemAddBlksToPool(memUint_t *pMemUint, void *memAddr, int blkAddNum, int blkSize)
{
    void **p_link;
    void *p_free_last;
    uint8_t *p_blk;

    if (memAddr == NULL)
    {
        return;
    }

    p_free_last = pMemUint->freeList;
    pMemUint->freeNum += blkAddNum;
    pMemUint->freeList = memAddr;

    p_link = (void **)memAddr;
    p_blk = (uint8_t *)memAddr;
    for (int i = 0; i < blkAddNum - 1; i++)
    {
        p_blk += blkSize;
        *p_link = (void *)p_blk;
        p_link = (void **)p_blk;
    }
    *p_link = p_free_last;
}

/* 初始化内存池 */
void MemPoolInit(memUint_t *pMemUint, void *memAddr, int blkNum, int blkSize)
{
    void **p_link;
    uint8_t *p_blk;

    pMemUint->freeNum = blkNum;
    pMemUint->blkSize = blkSize;
    pMemUint->freeList = memAddr;

    if (memAddr == NULL)
    {
        return;
    }

    p_link = (void **)memAddr;
    p_blk = (uint8_t *)memAddr;
    for (int i = 0; i < blkNum - 1; i++)
    {
        p_blk += blkSize;
        *p_link = (void *)p_blk;
        p_link = (void **)p_blk;
    }
    p_link = NULL;
}

/* 获取一块内存 */
void *MemGetBlk(memUint_t *pMemUint)
{
    void *p_blk;

    if (pMemUint->freeNum == 0)
    {
        return NULL;
    }

    p_blk = pMemUint->freeList;
    pMemUint->freeList = *(void **)p_blk;
    pMemUint->freeNum --;

    return p_blk;
}

/* 释放一块内存 */
int MemPutBlk(memUint_t *pMemUint, void *pMem)
{
    if (pMem == NULL)
    {
        return -1;
    }

    *(void **)pMem = pMemUint->freeList;
    pMemUint->freeList = pMem;
    pMemUint->freeNum ++;

    return 0;
}

