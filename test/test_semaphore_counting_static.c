/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-24     yangjie      the first version
 */

/*
 * 程序清单：信号量例程
 *
 * 该例程创建了一个动态信号量，初始化两个线程，线程1在count每计数10次时，
 * 发送一个信号量，线程2在接收信号量后，对number进行加1操作
 */
#include <rtthread.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define THREAD_PRIORITY         25
#define THREAD_TIMESLICE        5

/* 指向信号量的指针 */
static SemaphoreHandle_t static_sem = RT_NULL;
static StaticSemaphore_t xMutexBuffer;

ALIGN(RT_ALIGN_SIZE)
static char thread1_stack[1024];
static struct rt_thread thread1;
static void rt_thread1_entry(void *parameter)
{
    static rt_err_t result;
    static rt_uint8_t number = 0;
    while (1)
    {
        for (number = 0; number < 5; number++)
        {
            result = xSemaphoreTake(static_sem, portMAX_DELAY);
            if (result != pdPASS)
            {
                rt_kprintf("thread1 take a static semaphore, failed.\n");
                return;
            }
            else
            {
                rt_kprintf("thread1 take a static semaphore. number = %d\n", number);
            }
        }
        result = xSemaphoreTake(static_sem, 0);
        if (result != errQUEUE_EMPTY)
        {
            rt_kprintf("thread1 take a static semaphore. number = %d. Should not succeed.\n", number);
        }
        rt_thread_delay(rt_tick_from_millisecond(10000));
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread2_stack[1024];
static struct rt_thread thread2;
static void rt_thread2_entry(void *parameter)
{
    static rt_err_t result;
    static rt_uint8_t number = 0;
    while (1)
    {
        for (number = 0; number < 5; number++)
        {
            result = xSemaphoreGive(static_sem);
            if (result != pdPASS)
            {
                rt_kprintf("thread2 release a static semaphore, failed.\n");
                return;
            }
            else
            {
                rt_kprintf("thread2 release a static semaphore. number = %d\n", number);
            }
        }
        result = xSemaphoreGive(static_sem);
        if (result != errQUEUE_FULL)
        {
            rt_kprintf("thread2 release a static semaphore. number = %d. Should not succeed.\n", number);
        }
        rt_thread_delay(rt_tick_from_millisecond(10000));
    }
}

/* 信号量示例的初始化 */
int semaphore_counting_static()
{
    /* 创建一个动态信号量，初始值是0 */
    static_sem = xSemaphoreCreateCountingStatic(5, 0, &xMutexBuffer);
    if (static_sem == RT_NULL)
    {
        rt_kprintf("create static semaphore failed.\n");
        return -1;
    }
    rt_thread_init(&thread1,
                   "thread1",
                   rt_thread1_entry,
                   RT_NULL,
                   &thread1_stack[0],
                   sizeof(thread1_stack),
                   THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&thread1);

    rt_thread_init(&thread2,
                   "thread2",
                   rt_thread2_entry,
                   RT_NULL,
                   &thread2_stack[0],
                   sizeof(thread2_stack),
                   THREAD_PRIORITY - 1, THREAD_TIMESLICE);
    rt_thread_startup(&thread2);

    return 0;
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(semaphore_counting_static, semaphore sample);
