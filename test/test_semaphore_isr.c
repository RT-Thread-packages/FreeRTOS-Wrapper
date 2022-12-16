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
 * This demo creates a periodic hard timer which releases a semaphore when timeout
 * A thread blocks on the semaphore
 *
 */

#include <rtthread.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define THREAD_PRIORITY         (FINSH_THREAD_PRIORITY + 1)
#define THREAD_TIMESLICE        5

/* Semaphore handle */
static SemaphoreHandle_t dynamic_sem = RT_NULL;
static rt_timer_t timer1;

#ifdef rt_align
rt_align(RT_ALIGN_SIZE)
#else
ALIGN(RT_ALIGN_SIZE)
#endif
static char thread1_stack[1024];
static struct rt_thread thread1;
static void rt_thread1_entry(void *parameter)
{
    static rt_err_t result;
    while (1)
    {
        result = xSemaphoreTake(dynamic_sem, portMAX_DELAY);
        if (result != pdPASS)
        {
            rt_kprintf("thread1 take a dynamic semaphore, failed.\n");
            return;
        }
        else
        {
            rt_kprintf("thread1 take a dynamic semaphore, succeeded.\n");
        }
    }
}

static void timeout(void *parameter)
{
    rt_kprintf("timer isr give semaphore\n");
    xSemaphoreGiveFromISR(dynamic_sem, RT_NULL);
}

int semaphore_isr()
{
    /* Create a binary semaphore */
    dynamic_sem = xSemaphoreCreateBinary();
    if (dynamic_sem == RT_NULL)
    {
        rt_kprintf("create dynamic semaphore failed.\n");
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
    /* Create a hard timer with period of 5 seconds */
    timer1 = rt_timer_create("timer1", timeout, RT_NULL, rt_tick_from_millisecond(5000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    if (timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }
    return 0;
}

MSH_CMD_EXPORT(semaphore_isr, semaphore sample);
