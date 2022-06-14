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
 * Demo: recursive mutex
 *
 * This demo demonstrates statically creating a recursive mutex and using it to manage shared resources.
 *
 */

#include <rtthread.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define THREAD_PRIORITY         8
#define THREAD_TIMESLICE        5

/* mutex handler */
static SemaphoreHandle_t static_mutex = RT_NULL;
/* Buffer to store mutex structure */
static StaticSemaphore_t xMutexBuffer;
static rt_uint8_t number1, number2 = 0;

ALIGN(RT_ALIGN_SIZE)
static char thread1_stack[1024];
static struct rt_thread thread1;
static void rt_thread_entry1(void *parameter)
{
    while (1)
    {
        /* pending the mutex */
        xSemaphoreTakeRecursive(static_mutex, portMAX_DELAY);
        /* protect and deal with public variables */
        number1++;
        rt_thread_mdelay(10);
        number2++;
        if (number1 != number2)
        {
            rt_kprintf("not protect.number1 = %d, mumber2 = %d \n", number1, number2);
        }
        else
        {
            rt_kprintf("mutex protect ,number1 = mumber2 is %d\n", number1);
        }
        /* release the mutex */
        xSemaphoreGiveRecursive(static_mutex);

        if (number1 >= 100)
        {
            vSemaphoreDelete(static_mutex);
            return;
        }
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread2_stack[1024];
static struct rt_thread thread2;
static void rt_thread_entry2(void *parameter)
{
    while (1)
    {
        xSemaphoreTakeRecursive(static_mutex, portMAX_DELAY);
        number1++;
        number2++;
        xSemaphoreGiveRecursive(static_mutex);

        if (number1 >= 50)
            return;
    }
}

int mutex_recursive_static(void)
{
    /* Create a recursive mutex statically */
    static_mutex = xSemaphoreCreateRecursiveMutexStatic(&xMutexBuffer);
    if (static_mutex == RT_NULL)
    {
        rt_kprintf("create static mutex failed.\n");
        return -1;
    }

    rt_thread_init(&thread1,
                   "thread1",
                   rt_thread_entry1,
                   RT_NULL,
                   &thread1_stack[0],
                   sizeof(thread1_stack),
                   THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&thread1);

    rt_thread_init(&thread2,
                   "thread2",
                   rt_thread_entry2,
                   RT_NULL,
                   &thread2_stack[0],
                   sizeof(thread2_stack),
                   THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&thread2);
    return 0;
}

MSH_CMD_EXPORT(mutex_recursive_static, mutex sample);
