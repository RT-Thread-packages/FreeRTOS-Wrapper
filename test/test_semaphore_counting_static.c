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
 * Demo: semaphore
 * This demo creates one counting semaphore statically
 * It creates two threads:
 *    1) thread #1: take the semaphore until its value reaches 0
 *    2) thread #2: give the semaphore until its value reaches maximum
 *
 */

#include <rtthread.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define THREAD_PRIORITY         25
#define THREAD_TIMESLICE        5

/* Semaphore handle */
static SemaphoreHandle_t static_sem = RT_NULL;
/* Buffer to store semaphore structure */
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
        /* Thread1 starts when thread2 is delayed. Semaphore value is 5. Should take it successfully for five times */
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
        /* Cannot take the semaphore for the sixth time because the value is 0 */
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
        /* Thread2 runs before thread1. The semaphore value is 0. Should give the semaphore 5 times successfully */
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
        /* Cannot give the semaphore for the sixth time because the max value is reached */
        result = xSemaphoreGive(static_sem);
        if (result != errQUEUE_FULL)
        {
            rt_kprintf("thread2 release a static semaphore. number = %d. Should not succeed.\n", number);
        }
        rt_thread_delay(rt_tick_from_millisecond(10000));
    }
}

int semaphore_counting_static()
{
    /* Create a counting semaphore statically. Max value is 5. Initial value is 0. */
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

MSH_CMD_EXPORT(semaphore_counting_static, semaphore sample);
