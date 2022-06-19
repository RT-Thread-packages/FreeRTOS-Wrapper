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

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#define TASK_PRIORITY         8

/* mutex handler */
static SemaphoreHandle_t static_mutex = NULL;
/* Buffer to store mutex structure */
static StaticSemaphore_t xMutexBuffer;
static TaskHandle_t TaskHandle1 = NULL;
static TaskHandle_t TaskHandle2 = NULL;
static BaseType_t number1, number2 = 0;

static void vTask1Code(void *pvParameters)
{
    while (1)
    {
        /* pending the mutex */
        xSemaphoreTakeRecursive(static_mutex, portMAX_DELAY);
        /* protect and deal with public variables */
        number1++;
        vTaskDelay(pdMS_TO_TICKS(10));
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

static void vTask2Code(void * pvParameters)
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
    if (static_mutex == NULL)
    {
        rt_kprintf("create static mutex failed.\n");
        return -1;
    }
    xTaskCreate( vTask1Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &TaskHandle1 );
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

MSH_CMD_EXPORT(mutex_recursive_static, mutex sample);
