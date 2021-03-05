/**
*  @file memory_mgr.c/h
*  @version 0.9
*  @date 2017-08-28
*
*  @brief
*   0.9: 基本功能，固定大小的内存块复用
*
*  @attention
*   没有做线程安全，使用时必须加锁
*
*
*  @copyright 2017 RoboMaster. All right reserved.
*
*/

#ifndef MEMORY_MGR_H
#define MEMORY_MGR_H

#include <stdint.h>


#ifndef NULL
    #define NULL    (void*)0
#endif

typedef struct
{
    void *freeList;
    uint16_t freeNum;
    uint16_t blkSize;
} memUint_t;

void MemPoolInit(memUint_t *pMemUint, void *memAddr, int blkNum, int blkSize);
void MemAddBlksToPool(memUint_t *pMemUint, void *memAddr, int blkAddNum, int blkSize);
void *MemGetBlk(memUint_t *pMemUint);
int MemPutBlk(memUint_t *pMemUint, void *pMem);


#endif
