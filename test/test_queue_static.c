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
 * Demo: queue
 *
 * This demo demonstrates statically creating a queue and using it to send data between two tasks.
 *
 */

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#define TASK_PRIORITY         (FINSH_THREAD_PRIORITY + 1)
#define QUEUE_LENGTH          10
#define ITEM_SIZE             sizeof( uint32_t )

/* queue handler */
static QueueHandle_t xQueue = NULL;
static StaticQueue_t xQueueBuffer;
static uint8_t ucQueueStorage[ QUEUE_BUFFER_SIZE( QUEUE_LENGTH, ITEM_SIZE) ];
static TaskHandle_t TaskHandle1 = NULL;
static TaskHandle_t TaskHandle2 = NULL;

static void vTask1Code(void *pvParameters)
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

static void vTask2Code(void * pvParameters)
{
    BaseType_t xReturn;
    uint32_t num = 0;
    while (1)
    {
        xReturn = xQueueSendToBack(xQueue, &num, 0);
        if (xReturn != pdPASS)
        {
            rt_kprintf("Task 2 send to queue failed\n");
        }
        num += 1;
        if (num >= 100)
        {
            return;
        }
    }
}

int queue_static(void)
{
    /* Create a queue statically */
    xQueue = xQueueCreateStatic( QUEUE_LENGTH, ITEM_SIZE, &( ucQueueStorage[ 0 ] ), &xQueueBuffer);
    if (xQueue == NULL)
    {
        rt_kprintf("create static queue failed.\n");
        return -1;
    }
    xTaskCreate( vTask1Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY + 1, &TaskHandle1 );
    if (TaskHandle1 == NULL)
    {
        rt_kprintf("Create task 1 failed\n");
        return -1;
    }
    xTaskCreate( vTask2Code, "Task2", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &TaskHandle2 );
    if (TaskHandle2 == NULL)
    {
        rt_kprintf("Create task 2 failed\n");
        return -1;
    }

    return 0;
}

MSH_CMD_EXPORT(queue_static, queue sample);
