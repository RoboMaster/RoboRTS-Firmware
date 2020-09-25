#ifndef MEM_MANG_H
#define MEM_MANG_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sys.h"

#define mem_printf(...)

#define HEAP_ASSERT(x) \
{                      \
    if(!(x))           \
    {                  \
        for(; ; );     \
    }                  \
}

#ifndef TOTAL_HEAP_SIZE
    #define TOTAL_HEAP_SIZE (1024 * 20)
#endif

/* must be power of 2, at least 8 */
#define BYTE_ALIGNMENT (8)
#define BYTE_ALIGNMENT_MASK (BYTE_ALIGNMENT - 1)
#define POINTER_SIZE_TYPE unsigned int

/* A few bytes might be lost to byte aligning the heap start address. */
#define ADJUSTED_HEAP_SIZE (TOTAL_HEAP_SIZE - BYTE_ALIGNMENT)

void *heap_malloc(uint32_t wanted_size);
void heap_free(void *pv);
uint32_t heap_get_free(void);
uint32_t heap_get_ever_free(void);
void heap_print_block(void);

#endif
