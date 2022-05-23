
/* Standard includes. */
#include <stdlib.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include <rtthread.h>
#include "task.h"
#include "queue.h"
#include "timers.h"

#define LOG_TAG              "TIMER"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#if ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 0 )
    #error configUSE_TIMERS must be set to 1 to make the xTimerPendFunctionCall() function available.
#endif

/* Lint e961 and e750 are suppressed as a MISRA exception justified because the
MPU ports require MPU_WRAPPERS_INCLUDED_FROM_API_FILE to be defined for the
header files above, but not in this file, in order to generate the correct
privileged Vs unprivileged linkage and placement. */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750. */


/* This entire source file will be skipped if the application is not configured
to include software timer functionality.  This #if is closed at the very bottom
of this file.  If you want to include software timer functionality then ensure
configUSE_TIMERS is set to 1 in FreeRTOSConfig.h. */
#if ( configUSE_TIMERS == 1 )

static volatile unsigned short tm_index = 0;

TimerHandle_t xTimerCreate( const char * const pcTimerName, const TickType_t xTimerPeriodInTicks, const UBaseType_t uxAutoReload, void * const pvTimerID, TimerCallbackFunction_t pxCallbackFunction ) /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
{
    char name[10] = {0}, *tname = (char *)pcTimerName;
    rt_timer_t obj = 0;
    if (pcTimerName == NULL)
    {
        sprintf(name,"tim%02d",((++tm_index)%100));
        tname = name;
    }
    if (xTimerPeriodInTicks > 0)
    {
        rt_uint8_t flag = (uxAutoReload == pdTRUE)?(RT_TIMER_FLAG_PERIODIC):(RT_TIMER_FLAG_ONE_SHOT);
        obj = rt_timer_create(tname,pxCallbackFunction,pvTimerID,xTimerPeriodInTicks,flag);
    }

    return obj;
}
/*-----------------------------------------------------------*/

BaseType_t xTimerGenericCommand( TimerHandle_t xTimer, const BaseType_t xCommandID, const TickType_t xOptionalValue, BaseType_t * const pxHigherPriorityTaskWoken, const TickType_t xTicksToWait )
{
    rt_timer_t obj = xTimer;

    rt_err_t err = RT_EOK;
    switch(xCommandID)
    {
    case tmrCOMMAND_START:
        err = rt_timer_start(obj);
        break;
    case tmrCOMMAND_STOP:
        err = rt_timer_stop(obj);
        break;
    case tmrCOMMAND_CHANGE_PERIOD:{
        rt_tick_t tick = xOptionalValue;
        err = rt_timer_control(obj,RT_TIMER_CTRL_SET_TIME,&tick);
        break;}
    case tmrCOMMAND_DELETE:
        err = rt_timer_delete(obj);
        break;
    }

    if (pxHigherPriorityTaskWoken) {
        *pxHigherPriorityTaskWoken = pdFAIL;
    }

    return (err==RT_EOK)?pdPASS:pdFAIL;
}
/*-----------------------------------------------------------*/

BaseType_t xTimerIsTimerActive( TimerHandle_t xTimer )
{
    rt_timer_t timer = (rt_timer_t)xTimer;
    if (!timer)
        return pdFALSE;
    if (!(timer->parent.flag & RT_TIMER_FLAG_ACTIVATED))
        return pdFALSE;
    return pdTRUE;
} /*lint !e818 Can't be pointer to const due to the typedef. */
/*-----------------------------------------------------------*/

/* This entire source file will be skipped if the application is not configured
to include software timer functionality.  If you want to include software timer
functionality then ensure configUSE_TIMERS is set to 1 in FreeRTOSConfig.h. */
#endif /* configUSE_TIMERS == 1 */

