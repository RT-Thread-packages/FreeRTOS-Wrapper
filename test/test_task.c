/*
 * Demo: task
 *
 * This demo creates two tasks. Task 1 is created dynamically and task 2 is created statically.
 * Task 2 increments a number than delays for 1 second.
 * After task 2 runs for 10 iterations, task 1 deletes task 2, then deletes itself.
 *
 */

#include <FreeRTOS.h>
#include <task.h>

#define TASK_PRIORITY         (FINSH_THREAD_PRIORITY + 1)

static TaskHandle_t TaskHandle1 = NULL;
static TaskHandle_t TaskHandle2 = NULL;
static StaticTask_t xTaskBuffer;
StackType_t xStack[configMINIMAL_STACK_SIZE * 2];
static BaseType_t num = 0;

static void vTask1Code(void * pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        rt_kprintf("Task 1 running\n");
        if (num > 10)
        {
            rt_kprintf("Delete task 2\n");
            vTaskDelete(TaskHandle2);
            rt_kprintf("Delete task 1\n");
            vTaskDelete(NULL);
            rt_kprintf("Should not reach here\n");
        }
    }
}

static void vTask2Code(void * pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        num += 1;
        rt_kprintf("Task 2 running, num = %d\n",num);
    }
}

int task_sample(void)
{
    xTaskCreate(vTask1Code, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &TaskHandle1);
    if (TaskHandle1 == NULL)
    {
        rt_kprintf("Create task 1 failed\n");
        return -1;
    }
    TaskHandle2 = xTaskCreateStatic(vTask2Code, "Task2", configMINIMAL_STACK_SIZE * 2, NULL, TASK_PRIORITY, xStack, &xTaskBuffer);
    if (TaskHandle2 == NULL)
    {
        rt_kprintf("Create task 2 failed\n");
        return -1;
    }

    return 0;
}

MSH_CMD_EXPORT(task_sample, task sample);
