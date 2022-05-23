
#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <rthw.h>
#include <rtthread.h>

#define MT2625_QUEUE_DEBUG
#ifdef MT2625_QUEUE_DEBUG
#define LOG_TAG              "QUEUE"
#define LOG_LVL              LOG_LVL_ERROR // LOG_LVL_ERROR | LOG_LVL_DBG
#include <ulog.h>
#else
#define LOG_D(...)
#define LOG_I(...)
#define LOG_W(...)
#define LOG_E(...)
#endif

/* Lint e961 and e750 are suppressed as a MISRA exception justified because the
MPU ports require MPU_WRAPPERS_INCLUDED_FROM_API_FILE to be defined for the
header files above, but not in this file, in order to generate the correct
privileged Vs unprivileged linkage and placement. */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750. */


/* Semaphores do not actually store or copy data, so have an item size of
zero. */
#define queueSEMAPHORE_QUEUE_ITEM_LENGTH ( ( UBaseType_t ) 0 )
#define queueMUTEX_GIVE_BLOCK_TIME       ( ( TickType_t ) 0U )

static volatile unsigned short mq_index = 0;
static volatile unsigned short ms_index = 0;
static volatile unsigned short mx_index = 0;
static volatile unsigned short mr_index = 0;

extern rt_err_t rt_fmq_delete(rt_mailbox_t mb);
extern rt_err_t rt_fmq_recv(rt_mailbox_t mb, void *value, rt_int32_t peek, rt_int32_t timeout);
extern rt_err_t rt_fmq_send(rt_mailbox_t mb, void *value, rt_int32_t pos, rt_int32_t timeout);
extern rt_mailbox_t rt_fmq_create(const char *name, rt_size_t item, rt_size_t size, rt_uint8_t flag);

/*-----------------------------------------------------------*/

BaseType_t xQueueGenericReset( QueueHandle_t xQueue, BaseType_t xNewQueue )
{
    LOG_E("queue reset, TODO");
    return pdPASS;
}

/*-----------------------------------------------------------*/
extern uint32_t get_current_irq();
QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, const uint8_t ucQueueType )
{
    char name[10] = {0};
    rt_object_t obj = 0;

    if (uxItemSize <= 0 || uxQueueLength <= 0)
    {
        if (ucQueueType == queueQUEUE_TYPE_MUTEX || ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX)
        {
            if (ucQueueType == queueQUEUE_TYPE_MUTEX)
            {
                sprintf(name,"mux%02d",((++mx_index)%100));
                // obj = (rt_object_t)rt_mutex_create(name,RT_IPC_FLAG_PRIO);
                obj = (rt_object_t)rt_sem_create(name, 1, RT_IPC_FLAG_PRIO);
                LOG_D("mux:%p, name:%s", obj, name);
            }
            else
            {
                sprintf(name,"rmx%02d",((++mr_index)%100));
                obj = (rt_object_t)rt_mutex_create(name,RT_IPC_FLAG_PRIO);
            }
        }
        else if (ucQueueType == queueQUEUE_TYPE_BINARY_SEMAPHORE)
        {
            sprintf(name,"Bsem%02d",((++ms_index)%100));
            obj = (rt_object_t)rt_sem_create(name,0,RT_IPC_FLAG_PRIO);
            LOG_D("Bsem:%p, name:%s", obj, name);
        }
        else if (ucQueueType == queueQUEUE_TYPE_COUNTING_SEMAPHORE)
        {
            sprintf(name,"sem%02d",((++ms_index)%100));
            obj = (rt_object_t)rt_sem_create(name,0,RT_IPC_FLAG_PRIO);
        }
    }
    else
    {
        sprintf(name,"fmq%02d",((++mq_index)%100));
        obj = (rt_object_t)rt_fmq_create(name,uxItemSize,uxQueueLength,RT_IPC_FLAG_PRIO);
    }

    return obj;
}
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )

    QueueHandle_t xQueueCreateMutex( const uint8_t ucQueueType )
    {
        QueueHandle_t obj;
        obj = xQueueGenericCreate(1, 0, ucQueueType);

        return obj;
    }

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) )

    void* xQueueGetMutexHolder( QueueHandle_t xSemaphore )
    {
        LOG_E("Get mutex owner, TODO");
        return NULL;
    } /*lint !e818 xSemaphore cannot be a pointer to const because it is a typedef. */

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t xQueueGiveMutexRecursive( QueueHandle_t xMutex )
    {
        return xQueueGenericSend(xMutex,0,0,queueSEND_TO_BACK);
    }

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t xQueueTakeMutexRecursive( QueueHandle_t xMutex, TickType_t xTicksToWait )
    {
        return xQueueGenericReceive(xMutex,0,xTicksToWait,pdFALSE);
    }

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( configUSE_COUNTING_SEMAPHORES == 1 )

    QueueHandle_t xQueueCreateCountingSemaphore( const UBaseType_t uxMaxCount, const UBaseType_t uxInitialCount )
    {
        return xQueueGenericCreate( uxMaxCount, queueSEMAPHORE_QUEUE_ITEM_LENGTH, queueQUEUE_TYPE_COUNTING_SEMAPHORE );
    }

#endif /* configUSE_COUNTING_SEMAPHORES */
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, const BaseType_t xCopyPosition )
{
    rt_object_t obj = xQueue;
    rt_err_t err = RT_EOK;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (obj->type == RT_Object_Class_Semaphore)
    {
        LOG_D("-> release sem[0x%08x], thread:%s", obj, rt_thread_self()->name);
        err = rt_sem_release((rt_sem_t)obj);
    }
    else if (obj->type == RT_Object_Class_Mutex)
    {
        LOG_D("-> release mutex, thread:%s", rt_thread_self()->name);
        err = rt_mutex_release((rt_mutex_t)obj);
    }
    else
    {
        LOG_D("-> send fast mq, thread:%s", rt_thread_self()->name);
        err = rt_fmq_send((rt_mailbox_t)obj,(void *)pvItemToQueue,xCopyPosition,xTicksToWait);
    }

    return (err==RT_EOK)?pdPASS:errQUEUE_FULL;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue, const void * const pvItemToQueue, BaseType_t * const pxHigherPriorityTaskWoken, const BaseType_t xCopyPosition )
{
    rt_object_t obj = xQueue;
    rt_err_t err = RT_EOK;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (obj->type == RT_Object_Class_Semaphore)
    {
        LOG_D("-> release sem[INT]");
        err = rt_sem_release((rt_sem_t)obj);
    }
    else if (obj->type == RT_Object_Class_Mutex)
    {
        LOG_E("release mutex in isr");
    }
    else
    {
        LOG_D("-> send fast mq[INT]");
        err = rt_fmq_send((rt_mailbox_t)obj,(void *)pvItemToQueue, xCopyPosition, 0);
    }

    if (pxHigherPriorityTaskWoken) *pxHigherPriorityTaskWoken = pdFALSE;
    return (err==RT_EOK)?pdPASS:errQUEUE_FULL;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue, BaseType_t * const pxHigherPriorityTaskWoken )
{
    return xQueueGenericSendFromISR(xQueue,0,pxHigherPriorityTaskWoken,queueSEND_TO_BACK);
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait, const BaseType_t xJustPeeking )
{
    rt_object_t obj = xQueue;
    rt_err_t err = RT_EOK;
    rt_int32_t timeout;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (xTicksToWait == portMAX_DELAY)
        timeout = RT_WAITING_FOREVER;
    else
        timeout = xTicksToWait;

    if (obj->type == RT_Object_Class_Semaphore)
    {
        LOG_D("┌- try to take sem[0x%08x]", obj);
        err = rt_sem_take((rt_sem_t)obj, timeout);
        LOG_D("└> taken sem[0x%08x], err=%d, thread:%s", obj, err, rt_thread_self()->name);
    }
    else if (obj->type == RT_Object_Class_Mutex)
    {
        LOG_D("┌- try to take mutex");
        err = rt_mutex_take((rt_mutex_t)obj, timeout);
        LOG_D("└> taken mutex, err=%d, thread:%s", err, rt_thread_self()->name);
    }
    else
    {
        LOG_D("┌- try to send fast mq");
        err = rt_fmq_recv((rt_mailbox_t)obj,(void *)pvBuffer,xJustPeeking,timeout);
        LOG_D("└> recved fast mq, err=%d, thread:%s", err, rt_thread_self()->name);
    }

    return (err==RT_EOK)?pdPASS:errQUEUE_EMPTY;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue, void * const pvBuffer, BaseType_t * const pxHigherPriorityTaskWoken )
{
    rt_object_t obj = xQueue;
    rt_err_t err = RT_EOK;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (obj->type == RT_Object_Class_Semaphore)
    {
        LOG_D("┌- try to take sem[INT]");
        err = rt_sem_take((rt_sem_t)obj, 0);
        LOG_D("└> taken sem, err=%d", err);
    }
    else if (obj->type == RT_Object_Class_Mutex)
    {
        LOG_E("take mutex in isr");
    }
    else
    {
        LOG_D("┌- try to send fast mq[INT]");
        err = rt_fmq_recv((rt_mailbox_t)obj,(void *)pvBuffer, pdFALSE, 0);
        LOG_D("└> recved fast mq, err=%d", err);
    }

    if (pxHigherPriorityTaskWoken) *pxHigherPriorityTaskWoken = pdFALSE;

    return (err==RT_EOK)?pdPASS:errQUEUE_EMPTY;
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaiting( const QueueHandle_t xQueue )
{
    rt_object_t obj = xQueue;
    unsigned portBASE_TYPE count = 0;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (obj->type == RT_Object_Class_Semaphore)
        count = ((rt_sem_t)obj)->value;
    else if (obj->type == RT_Object_Class_Mutex)
        count = ((rt_mutex_t)obj)->value;
    else
        count = ((rt_mailbox_t)obj)->entry;

    return count;
} /*lint !e818 Pointer cannot be declared const as xQueue is a typedef not pointer. */
/*-----------------------------------------------------------*/

UBaseType_t uxQueueSpacesAvailable( const QueueHandle_t xQueue )
{
    UBaseType_t uxReturn = 0;
    rt_object_t obj = xQueue;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (obj->type == RT_Object_Class_MailBox)
    {
        rt_ubase_t level;
        struct rt_mailbox *mb = (struct rt_mailbox *)obj;

        rt_enter_critical();
        uxReturn = mb->size - mb->entry;
        rt_exit_critical();
    }

    return uxReturn;
} /*lint !e818 Pointer cannot be declared const as xQueue is a typedef not pointer. */
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaitingFromISR( const QueueHandle_t xQueue )
{
    UBaseType_t uxReturn;

    uxReturn = uxQueueMessagesWaiting(xQueue);

    return uxReturn;
} /*lint !e818 Pointer cannot be declared const as xQueue is a typedef not pointer. */
/*-----------------------------------------------------------*/

void vQueueDelete( QueueHandle_t xQueue )
{
    rt_object_t obj = xQueue;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (obj->type == RT_Object_Class_Semaphore)
        rt_sem_delete((rt_sem_t)obj);
    else if (obj->type == RT_Object_Class_Mutex)
        rt_mutex_delete((rt_mutex_t)obj);
    else
        rt_fmq_delete((rt_mailbox_t)obj);
}
/*-----------------------------------------------------------*/

void *uxQueueGetTasksWaitingToReceive(QueueHandle_t xQueue)
{
    // LOG_E("F:%s;L:%d; xQueue:%p TODO", __FUNCTION__, __LINE__, xQueue);

    rt_object_t obj = xQueue;
    rt_list_t *list;
    rt_thread_t thread;

    if (!obj)
    {
        LOG_E("In param (xQueue) is NULL!");
        configASSERT(obj);
    }

    if (obj->type == RT_Object_Class_Semaphore)
    {
        rt_kprintf("type sem\n");
        list = &(((rt_sem_t)obj)->parent.suspend_thread);
    }
    else if (obj->type == RT_Object_Class_Mutex)
    {
        rt_kprintf("type mutex\n");
        list = &(((rt_mutex_t)obj)->parent.suspend_thread);
    }
    else
    {
        rt_kprintf("type others\n");
        list = &(((rt_mailbox_t)obj)->parent.suspend_thread);
    }

    rt_enter_critical();
    thread = rt_list_entry(list->next, struct rt_thread, tlist);
    rt_exit_critical();

    return thread;
}
