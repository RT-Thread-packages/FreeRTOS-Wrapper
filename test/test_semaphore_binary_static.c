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
 * This demo creates one binary semaphore statically
 * It creates two threads:
 *    1) thread #1: give the semaphore
 *    2) thread #2: take the semaphore
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
    static rt_uint8_t count = 0;

    while (1)
    {
        if (count <= 100)
        {
            count++;
        }
        else
            return;

        /* Release the semaphore when count is incremented by 10 */
        if (0 == (count % 10))
        {
            rt_kprintf("thread1 release a static semaphore.\n");
            result = xSemaphoreGive(static_sem);
            if (result != pdPASS)
            {
                rt_kprintf("thread2 take a static semaphore, failed.\n");
                return;
            }
        }
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
        /* Block on the semaphore indefinitely. Increment number after taking the semaphore */
        result = xSemaphoreTake(static_sem, portMAX_DELAY);
        if (result != pdPASS)
        {
            rt_kprintf("thread2 take a static semaphore, failed.\n");
            return;
        }
        else
        {
            number++;
            rt_kprintf("thread2 take a static semaphore. number = %d\n", number);
        }
    }
}

int semaphore_binary_static()
{
    /* Create a binary semaphore statically */
    static_sem = xSemaphoreCreateBinaryStatic(&xMutexBuffer);
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

MSH_CMD_EXPORT(semaphore_binary_static, semaphore sample);
