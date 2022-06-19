/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-24     yangjie      the first version
 * 2020-10-17     Meco Man     translate to English comment
 */

/*
 * Demo: heap
 *
 * This demo demonstrates allocating memory from the heap
 *
 */

#include <FreeRTOS.h>

// Set to 1 if using heap_5.c
#define USE_HEAP_5     0

#if USE_HEAP_5 == 1
rt_uint8_t heap[512];
HeapRegion_t heap_regions[] =
{
    {heap, 512},
    {NULL, 0}
};
#endif

int heap_sample(void)
{
#if USE_HEAP_5 == 1
    vPortDefineHeapRegions(heap_regions);
#endif
    rt_uint8_t *ptr = pvPortMalloc(128);
    if (ptr == RT_NULL)
    {
        rt_kprintf("Memory allocation failed.\n");
        return -1;
    }
    for (rt_uint8_t i = 0; i < 128; i++)
    {
        ptr[i] = i;
    }
    vPortFree(ptr);
    return 0;
}

MSH_CMD_EXPORT(heap_sample, heap sample);
