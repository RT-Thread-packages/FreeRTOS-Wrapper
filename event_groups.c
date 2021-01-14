
/* Standard includes. */
#include <stdlib.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"

#include <rtthread.h>

#define LOG_TAG              "EVENT"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

/* Lint e961 and e750 are suppressed as a MISRA exception justified because the
MPU ports require MPU_WRAPPERS_INCLUDED_FROM_API_FILE to be defined for the
header files above, but not in this file, in order to generate the correct
privileged Vs unprivileged linkage and placement. */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750. */
/*-----------------------------------------------------------*/

static volatile unsigned short ev_index = 0;

EventGroupHandle_t xEventGroupCreate( void )
{
    char name[10] = {0};
    sprintf(name,"evt%02d",((++ev_index)%100));

    rt_event_t obj = rt_event_create(name,RT_IPC_FLAG_PRIO);

    return obj;
}
void vEventGroupDelete( EventGroupHandle_t xEventGroup )
{
    rt_event_t obj = xEventGroup;

    rt_event_delete(obj);
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupSetBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet )
{
    rt_event_t obj = xEventGroup;
    rt_err_t err;

    LOG_D("-> event 0x%08x, thread:%s", uxBitsToSet, rt_thread_self()->name);

    err = rt_event_send(obj, uxBitsToSet);

    return (err!=RT_EOK)?0:obj->set;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupClearBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear )
{
    rt_event_t obj = xEventGroup;
    rt_uint32_t recved = 0;
    rt_err_t err;

    LOG_E("!! clean event 0x%08x, thread:%s", uxBitsToClear, rt_thread_self()->name);

    err = rt_event_recv(obj,uxBitsToClear,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,0,&recved);

    return (err!=RT_EOK)?0:recved;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupWaitBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait )
{
    rt_event_t obj = xEventGroup;
    rt_uint32_t recved = 0;
    rt_uint8_t option = 0;

    if (xClearOnExit == pdTRUE)
        option |= RT_EVENT_FLAG_CLEAR;
    if (xWaitForAllBits == pdTRUE)
        option |= RT_EVENT_FLAG_AND;
    else
        option |= RT_EVENT_FLAG_OR;

    rt_err_t err;

    LOG_D("┌- try to recv event[0x%08x]", obj);
    err = rt_event_recv(obj,uxBitsToWaitFor,option,xTicksToWait,&recved);
    LOG_D("└> recved event[0x%08x], err=%d, thread:%s", obj, err, rt_thread_self()->name);

    return (err!=RT_EOK)?0:recved;
}
/*-----------------------------------------------------------*/
