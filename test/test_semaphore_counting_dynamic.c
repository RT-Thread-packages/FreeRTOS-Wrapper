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
 * This demo creates one counting semaphore dynamically
 * It creates two tasks:
 *    1) task #1: take the semaphore until its value reaches 0
 *    2) task #2: give the semaphore until its value reaches maximum
 *
 */

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#define TASK_PRIORITY         (FINSH_THREAD_PRIORITY + 1)

/* Semaphore handle */
static SemaphoreHandle_t dynamic_sem = NULL;
static TaskHandle_t TaskHandle1 = NULL;
static TaskHandle_t TaskHandle2 = NULL;

static void vTask1Code(void *pvParameters)
{
    static BaseType_t result;
    static BaseType_t number = 0;
    while (1)
    {
        /* Task1 starts when task2 is delayed. Semaphore value is 5. Should take it successfully for five times */
        for (number = 0; number < 5; number++)
        {
            result = xSemaphoreTake(dynamic_sem, portMAX_DELAY);
            if (result != pdPASS)
            {
                rt_kprintf("task1 take a dynamic semaphore, failed.\n");
                return;
            }
            else
            {
                rt_kprintf("task1 take a dynamic semaphore. number = %d\n", number);
            }
        }
        /* Cannot take the semaphore for the sixth time because the value is 0 */
        result = xSemaphoreTake(dynamic_sem, 0);
        if (result != errQUEUE_EMPTY)
        {
            rt_kprintf("task1 take a dynamic semaphore. number = %d. Should not succeed.\n", number);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void vTask2Code(void * pvParameters)
{
    static BaseType_t result;
    static BaseType_t number = 0;
    while (1)
    {
        /* Task2 runs before task1. The semaphore value is 0. Should give the semaphore 5 times successfully */
        for (number = 0; number < 5; number++)
        {
            result = xSemaphoreGive(dynamic_sem);
            if (result != pdPASS)
            {
                rt_kprintf("task2 release a dynamic semaphore, failed.\n");
                return;
            }
            else
            {
                rt_kprintf("task2 release a dynamic semaphore. number = %d\n", number);
            }
        }
        /* Cannot give the semaphore for the sixth time because the max value is reached */
        result = xSemaphoreGive(dynamic_sem);
        if (result != errQUEUE_FULL)
        {
            rt_kprintf("task2 release a dynamic semaphore. number = %d. Should not succeed.\n", number);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

int semaphore_counting_dynamic()
{
    /* Create a counting semaphore dynamically. Max value is 5. Initial value is 0. */
    dynamic_sem = xSemaphoreCreateCounting(5, 0);
    if (dynamic_sem == NULL)
    {
        rt_kprintf("create dynamic semaphore failed.\n");
        return -1;
    }
    xTaskCreate( vTask2Code, "Task2", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY + 1, &TaskHandle2 );
    if (TaskHandle2 == NULL)
    {
        rt_kprintf("Create task 2 failed\n");
        return -1;
    }
    xTaskCreate( vTask1Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &TaskHandle1 );
    if (TaskHandle1 == NULL)
    {
        rt_kprintf("Create task 1 failed\n");
        return -1;
    }

    return 0;
}

MSH_CMD_EXPORT(semaphore_counting_dynamic, semaphore sample);
