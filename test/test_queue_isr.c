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
 * Demo: queue
 * This demo creates a periodic hard timer which sends data to a queue when timeout
 * A thread blocks to receive from the queue
 *
 */

#include <rtthread.h>
#include <FreeRTOS.h>
#include <queue.h>

#define THREAD_PRIORITY         (FINSH_THREAD_PRIORITY + 1)
#define THREAD_TIMESLICE        5
#define QUEUE_LENGTH            1
#define ITEM_SIZE               sizeof( uint32_t )

static QueueHandle_t xQueue = NULL;
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
    BaseType_t xReturn;
    uint32_t num = 0;
    while (1)
    {
        xReturn = xQueueReceive(xQueue, &num, portMAX_DELAY);
        if (xReturn != pdPASS)
        {
            rt_kprintf("Task 1 receive from queue failed\n");
        }
        else
        {
            rt_kprintf("Task 1 receive data %d from queue\n", num);
        }
        if (num >= 100)
        {
            return;
        }
    }
}

static void timeout(void *parameter)
{
    static uint32_t num = 0;
    rt_kprintf("timer isr send data %d to queue\n", num);
    xQueueSendToBackFromISR(xQueue, &num, NULL);
    num += 1;
}

int queue_isr()
{
    xQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
    if (xQueue == NULL)
    {
        rt_kprintf("create queue failed.\n");
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

MSH_CMD_EXPORT(queue_isr, queue sample);
