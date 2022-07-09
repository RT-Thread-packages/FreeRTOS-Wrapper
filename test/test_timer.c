/*
 * Demo: timer
 *
 * This demo demonstrates using timers
 *
 */

#include <FreeRTOS.h>
#include <timers.h>

static TimerHandle_t xTimer1 = NULL;
static TimerHandle_t xTimer2 = NULL;
static volatile UBaseType_t uxVariableToIncrement = 0;
static StaticTimer_t xTimerBuffer;

void prvTimerCallback(TimerHandle_t xTimer)
{
    UBaseType_t *puxVariableToIncrement;
    rt_kprintf("%s time out, period: %d\n", pcTimerGetName(xTimer), xTimerGetPeriod(xTimer));
    puxVariableToIncrement = (UBaseType_t *)pvTimerGetTimerID(xTimer);
    if (puxVariableToIncrement != NULL)
    {
        (*puxVariableToIncrement)++;
        rt_kprintf("Value: %d\n", *puxVariableToIncrement);
        if (*puxVariableToIncrement == 5)
        {
            xTimerChangePeriod(xTimer, pdMS_TO_TICKS(1000), 0);
        }
        else if (*puxVariableToIncrement == 10)
        {
            xTimerStop(xTimer, 0);
        }
    }
}

int timer_sample(void)
{
    xTimer1 = xTimerCreate("Timer 1", pdMS_TO_TICKS(1000), pdFALSE, NULL, prvTimerCallback);
    if (xTimer1 == NULL)
    {
        rt_kprintf("Create timer 1 failed");
        return -1;
    }
    xTimer2 = xTimerCreateStatic("Timer 2", pdMS_TO_TICKS(500), pdTRUE, (void *)&uxVariableToIncrement, prvTimerCallback, &xTimerBuffer);
    if (xTimer2 == NULL)
    {
        rt_kprintf("Create timer 2 failed");
        return -1;
    }
    xTimerStart(xTimer1, 0);
    xTimerStart(xTimer2, 0);

    return 0;
}

MSH_CMD_EXPORT(timer_sample, timer sample);
