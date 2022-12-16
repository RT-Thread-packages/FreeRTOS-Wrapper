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
 * This program demonstrates using task notification as a light weight event group
 *
 */

#include <rtthread.h>
#include <FreeRTOS.h>
#include <task.h>

#define TASK_PRIORITY         (FINSH_THREAD_PRIORITY + 1)
#define BIT_0                 ( 1 << 0 )
#define BIT_1                 ( 1 << 1 )

static TaskHandle_t TaskHandle1 = NULL;
static TaskHandle_t TaskHandle2 = NULL;
static TaskHandle_t TaskHandle3 = NULL;

static void vTask1Code(void * pvParameters)
{
    BaseType_t xResult;
    uint32_t ulNotificationValue;
    while (1)
    {
        xResult = xTaskNotifyWait( pdFALSE, BIT_0 | BIT_1, &ulNotificationValue, portMAX_DELAY);
        if (xResult == pdTRUE)
        {
            if (ulNotificationValue & BIT_0)
            {
                rt_kprintf("Task 1 received notification from task 2\n");
            }
            if (ulNotificationValue & BIT_1)
            {
                rt_kprintf("Task 1 received notification from task 3\n");
            }
        }
    }
}

void vTask2Code(void * pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        rt_kprintf("Task 2 send notification\n");
        xTaskNotify(TaskHandle1, BIT_0, eSetBits);
    }
}

void vTask3Code(void * pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        rt_kprintf("Task 3 send notification\n");
        xTaskNotify(TaskHandle1, BIT_1, eSetBits);
    }
}

int task_notification_event_group()
{
    xTaskCreate(vTask1Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY + 1, &TaskHandle1);
    if (TaskHandle1 == NULL)
    {
        rt_kprintf("Create task failed\n");
        return -1;
    }
    xTaskCreate(vTask2Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &TaskHandle2);
    if (TaskHandle2 == NULL)
    {
        rt_kprintf("Create task failed\n");
        return -1;
    }
    xTaskCreate(vTask3Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &TaskHandle3);
    if (TaskHandle3 == NULL)
    {
        rt_kprintf("Create task failed\n");
        return -1;
    }
    return 0;
}

MSH_CMD_EXPORT(task_notification_event_group, task notification sample);
