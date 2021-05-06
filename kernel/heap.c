#include <rtthread.h>

void *pvPortMalloc( size_t xWantedSize )
{
    return rt_malloc(xWantedSize);
}

void vPortFree( void *pv )
{
    rt_free(pv);
}

void *pvPortCalloc( size_t nmemb, size_t size )
{
    return rt_calloc(nmemb, size);
}

void *pvPortRealloc( void *pv, size_t size )
{
    return rt_realloc(pv, size);
}

void *pvPortMallocNC( size_t xWantedSize )
{
    extern void *mt_noncached_malloc(rt_size_t size);
    return mt_noncached_malloc(xWantedSize);
}
void vPortFreeNC( void *pv )
{
    extern void mt_noncached_free(void *ptr);;
    mt_noncached_free(pv);
}

size_t xPortGetFreeHeapSize( void )
{
    rt_uint32_t total, used, max;
    rt_memory_info(&total, &used, &max);

    return total - used;
}

size_t xPortGetMinimumEverFreeHeapSize( void )
{
    rt_uint32_t total, used, max;
    rt_memory_info(&total, &used, &max);

    return total - max;
}
