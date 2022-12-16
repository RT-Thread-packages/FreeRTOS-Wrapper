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
 *
 * Demo: task notification
 * This program demonstrates using task notification as a light weight binary semaphore
 *
 */

#include <rtthread.h>
#include <FreeRTOS.h>
#include <task.h>

#define TASK_PRIORITY         (FINSH_THREAD_PRIORITY + 1)

static TaskHandle_t xHandle = NULL;
static rt_timer_t timer1;

static void vTask1Code(void * pvParameters)
{
    while (1)
    {
        uint32_t ulNotificationValue;
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue == 1)
        {
            rt_kprintf("Task received notification\n");
        }
    }
}

static void timeout(void *parameter)
{
    rt_kprintf("Timer interrupt generated\n");
    vTaskNotifyGiveFromISR(xHandle, NULL);
}

int task_notification_binary_semaphore()
{
    xTaskCreate(vTask1Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &xHandle);
    if (xHandle == NULL)
    {
        rt_kprintf("Create task failed\n");
        return -1;
    }
    /* Create a hard timer with period of 1 second */
    timer1 = rt_timer_create("timer1", timeout, RT_NULL, rt_tick_from_millisecond(1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    if (timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }
    return 0;
}

MSH_CMD_EXPORT(task_notification_binary_semaphore, task notification sample);
