/*
 * FreeRTOS Kernel V10.4.6
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

/* Semaphores do not actually store or copy data, so have an item size of
 * zero. */
#define queueSEMAPHORE_QUEUE_ITEM_LENGTH    ( ( UBaseType_t ) 0 )
#define queueMUTEX_GIVE_BLOCK_TIME          ( ( TickType_t ) 0U )

typedef struct QueueDefinition
{
    struct rt_ipc_object *rt_ipc;
} xQUEUE;
typedef xQUEUE Queue_t;

static volatile rt_uint8_t mutex_index = 0;
static volatile rt_uint8_t sem_index = 0;
static volatile rt_uint8_t queue_index = 0;

/*-----------------------------------------------------------*/

BaseType_t xQueueGenericReset( QueueHandle_t xQueue,
                               BaseType_t xNewQueue )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;

    configASSERT( pxQueue );

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );

    if ( type == RT_Object_Class_Semaphore )
    {
        rt_sem_control( ( rt_sem_t ) pipc, RT_IPC_CMD_RESET, ( void * ) 0);
    }
    else if ( type == RT_Object_Class_MessageQueue )
    {
        rt_mq_control( ( rt_mq_t ) pipc, RT_IPC_CMD_RESET, RT_NULL );
    }

    return pdPASS;
}
/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    QueueHandle_t xQueueGenericCreateStatic( const UBaseType_t uxQueueLength,
                                             const UBaseType_t uxItemSize,
                                             uint8_t * pucQueueStorage,
                                             StaticQueue_t * pxStaticQueue,
                                             const uint8_t ucQueueType )
    {
        Queue_t * pxNewQueue = NULL;
        char name[RT_NAME_MAX] = {0};

        /* The StaticQueue_t structure and the queue storage area must be
         * supplied. */
        configASSERT( pxStaticQueue );

        if( ( uxQueueLength > ( UBaseType_t ) 0 ) &&
            ( pxStaticQueue != NULL ) &&

            /* A queue storage area should be provided if the item size is not 0, and
             * should not be provided if the item size is 0. */
            ( !( ( pucQueueStorage != NULL ) && ( uxItemSize == 0 ) ) ) &&
            ( !( ( pucQueueStorage == NULL ) && ( uxItemSize != 0 ) ) ) )
        {
            if ( ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX || ucQueueType == queueQUEUE_TYPE_MUTEX )
            {
                rt_snprintf( name, RT_NAME_MAX, "mutex%02d", mutex_index++ );
                rt_mutex_init( ( rt_mutex_t ) &( ( StaticSemaphore_t * ) pxStaticQueue )->ipc_obj.mutex, name, RT_IPC_FLAG_PRIO );
            }
            else if ( ucQueueType == queueQUEUE_TYPE_BINARY_SEMAPHORE || ucQueueType == queueQUEUE_TYPE_COUNTING_SEMAPHORE )
            {
                rt_snprintf( name, RT_NAME_MAX, "sem%02d", sem_index++ );
                rt_sem_init( ( rt_sem_t ) &( ( StaticSemaphore_t * ) pxStaticQueue )->ipc_obj.semaphore, name, 0, RT_IPC_FLAG_PRIO );
                ( ( StaticSemaphore_t * ) pxStaticQueue )->ipc_obj.semaphore.max_value = uxQueueLength;
            }
            else if ( ucQueueType == queueQUEUE_TYPE_BASE )
            {
                rt_snprintf( name, RT_NAME_MAX, "queue%02d", queue_index++ );
                rt_mq_init( &( pxStaticQueue->ipc_obj ), name, pucQueueStorage, uxItemSize, QUEUE_BUFFER_SIZE( uxQueueLength, uxItemSize ), RT_IPC_FLAG_PRIO );
            }
            else
            {
                return pxNewQueue;
            }
            pxStaticQueue->rt_ipc = ( struct rt_ipc_object * ) &pxStaticQueue->ipc_obj;
            pxNewQueue = ( QueueHandle_t ) pxStaticQueue;
        }

        return pxNewQueue;
    }

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

    QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength,
                                       const UBaseType_t uxItemSize,
                                       const uint8_t ucQueueType )
    {
        Queue_t * pxNewQueue = NULL;
        char name[RT_NAME_MAX] = {0};
        struct rt_ipc_object * pipc = RT_NULL;

        if( ( uxQueueLength > ( UBaseType_t ) 0 ) &&
            /* Check for multiplication overflow. */
            ( ( SIZE_MAX / uxQueueLength ) >= uxItemSize ) &&
            /* Check for addition overflow. */
            ( ( SIZE_MAX - sizeof( Queue_t ) ) >= ( uxQueueLength * uxItemSize ) ) )
        {
            pxNewQueue = ( Queue_t * ) RT_KERNEL_MALLOC( sizeof( Queue_t ) );
            if ( pxNewQueue == NULL )
            {
                return ( QueueHandle_t ) pxNewQueue;
            }
            if ( ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX || ucQueueType == queueQUEUE_TYPE_MUTEX )
            {
                rt_snprintf( name, RT_NAME_MAX, "mutex%02d", mutex_index++ );
                pipc = ( struct rt_ipc_object * ) rt_mutex_create( name, RT_IPC_FLAG_PRIO );
            }
            else if ( ucQueueType == queueQUEUE_TYPE_BINARY_SEMAPHORE || ucQueueType == queueQUEUE_TYPE_COUNTING_SEMAPHORE )
            {
                rt_snprintf( name, RT_NAME_MAX, "sem%02d", sem_index++ );
                pipc = ( struct rt_ipc_object * ) RT_KERNEL_MALLOC( sizeof( struct rt_semaphore_wrapper ) );
                if ( pipc != RT_NULL )
                {
                    rt_sem_init( ( rt_sem_t ) pipc, name, 0, RT_IPC_FLAG_PRIO );
                    ( ( struct rt_semaphore_wrapper * ) pipc )->max_value = uxQueueLength;
                    /* Mark as dynamic so we can distinguish in vQueueDelete */
                    pipc->parent.type &= ~RT_Object_Class_Static;
                }
            }
            else if ( ucQueueType == queueQUEUE_TYPE_BASE )
            {
                rt_snprintf( name, RT_NAME_MAX, "queue%02d", queue_index++ );
                pipc = ( struct rt_ipc_object * ) rt_mq_create( name, uxItemSize, uxQueueLength, RT_IPC_FLAG_PRIO);
            }

            if ( pipc == RT_NULL )
            {
                RT_KERNEL_FREE( pxNewQueue );
                return NULL;
            }
            pxNewQueue->rt_ipc = pipc;
        }

        return ( QueueHandle_t ) pxNewQueue;
    }

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if ( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )

    QueueHandle_t xQueueCreateMutex( const uint8_t ucQueueType )
    {
        QueueHandle_t xNewQueue;
        const UBaseType_t uxMutexLength = ( UBaseType_t ) 1, uxMutexSize = ( UBaseType_t ) 0;

        xNewQueue = xQueueGenericCreate( uxMutexLength, uxMutexSize, ucQueueType );
        return xNewQueue;
    }

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )

    QueueHandle_t xQueueCreateMutexStatic( const uint8_t ucQueueType,
                                           StaticQueue_t * pxStaticQueue )
    {
        QueueHandle_t xNewQueue;
        const UBaseType_t uxMutexLength = ( UBaseType_t ) 1, uxMutexSize = ( UBaseType_t ) 0;

        xNewQueue = xQueueGenericCreateStatic( uxMutexLength, uxMutexSize, NULL, pxStaticQueue, ucQueueType );

        return xNewQueue;
    }

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) )

    TaskHandle_t xQueueGetMutexHolder( QueueHandle_t xSemaphore )
    {
        TaskHandle_t pxReturn;
        struct rt_ipc_object *pipc;
        rt_uint8_t type;
        rt_base_t level;

        configASSERT( xSemaphore );

        pipc = xSemaphore->rt_ipc;
        RT_ASSERT( pipc != RT_NULL );
        type = rt_object_get_type( &pipc->parent );

        if ( type == RT_Object_Class_Mutex )
        {
            level = rt_hw_interrupt_disable();
            pxReturn = ( TaskHandle_t ) ( ( rt_mutex_t ) pipc )->owner;
            rt_hw_interrupt_enable( level );
        }
        else
        {
            pxReturn = NULL;
        }

        return pxReturn;
    }

#endif /* if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) )

    TaskHandle_t xQueueGetMutexHolderFromISR( QueueHandle_t xSemaphore )
    {
        return xQueueGetMutexHolder( xSemaphore );
    }

#endif /* if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t xQueueGiveMutexRecursive( QueueHandle_t xMutex )
    {
        Queue_t * const pxMutex = ( Queue_t * ) xMutex;
        configASSERT( pxMutex );
        return xQueueGenericSend( pxMutex, NULL, queueMUTEX_GIVE_BLOCK_TIME, queueSEND_TO_BACK );
    }

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t xQueueTakeMutexRecursive( QueueHandle_t xMutex,
                                         TickType_t xTicksToWait )
    {
        Queue_t * const pxMutex = ( Queue_t * ) xMutex;
        configASSERT( pxMutex );
        return xQueueSemaphoreTake( pxMutex, xTicksToWait );
    }

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )

    QueueHandle_t xQueueCreateCountingSemaphoreStatic( const UBaseType_t uxMaxCount,
                                                       const UBaseType_t uxInitialCount,
                                                       StaticQueue_t * pxStaticQueue )
    {
        QueueHandle_t xHandle = NULL;

        if( ( uxMaxCount != 0 ) &&
            ( uxInitialCount <= uxMaxCount ) )
        {
            xHandle = xQueueGenericCreateStatic( uxMaxCount, queueSEMAPHORE_QUEUE_ITEM_LENGTH, NULL, pxStaticQueue, queueQUEUE_TYPE_COUNTING_SEMAPHORE );

            if( xHandle != NULL )
            {
                ( ( rt_sem_t ) ( ( Queue_t * ) xHandle )->rt_ipc )->value = uxInitialCount;
            }
        }
        else
        {
            configASSERT( xHandle );
        }

        return xHandle;
    }

#endif /* ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )

    QueueHandle_t xQueueCreateCountingSemaphore( const UBaseType_t uxMaxCount,
                                                 const UBaseType_t uxInitialCount )
    {
        QueueHandle_t xHandle = NULL;

        if( ( uxMaxCount != 0 ) &&
            ( uxInitialCount <= uxMaxCount ) )
        {
            xHandle = xQueueGenericCreate( uxMaxCount, queueSEMAPHORE_QUEUE_ITEM_LENGTH, queueQUEUE_TYPE_COUNTING_SEMAPHORE );

            if( xHandle != NULL )
            {
                ( ( rt_sem_t ) ( ( Queue_t * ) xHandle )->rt_ipc )->value = uxInitialCount;
            }
        }
        else
        {
            configASSERT( xHandle );
        }

        return xHandle;
    }

#endif /* ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericSend( QueueHandle_t xQueue,
                              const void * const pvItemToQueue,
                              TickType_t xTicksToWait,
                              const BaseType_t xCopyPosition )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_base_t level;
    rt_err_t err = -RT_ERROR;

    configASSERT( pxQueue );
    #if ( INCLUDE_xTaskGetSchedulerState == 1 )
        {
            configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) && ( xTicksToWait != 0 ) ) );
        }
    #endif

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );
    if ( type == RT_Object_Class_Mutex )
    {
        err = rt_mutex_release( ( rt_mutex_t ) pipc );
    }
    else if ( type == RT_Object_Class_Semaphore )
    {
        level = rt_hw_interrupt_disable();
        if ( ( ( rt_sem_t ) pipc )->value < ( ( struct rt_semaphore_wrapper * ) pipc )->max_value )
        {
            err = rt_sem_release( ( rt_sem_t ) pipc );
        }
        rt_hw_interrupt_enable( level );
    }
    else if ( type == RT_Object_Class_MessageQueue )
    {
        if ( xCopyPosition == queueSEND_TO_BACK )
        {
            err = rt_mq_send_wait( ( rt_mq_t ) pipc, pvItemToQueue, ( ( rt_mq_t ) pipc )->msg_size, ( rt_int32_t ) xTicksToWait );
        }
        else if ( xCopyPosition == queueSEND_TO_FRONT )
        {
            // TODO: need to implement the timeout for LIFO
            err = rt_mq_urgent( ( rt_mq_t ) pipc, pvItemToQueue, ( ( rt_mq_t ) pipc )->msg_size );
        }
    }

    return rt_err_to_freertos( err );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue,
                                     const void * const pvItemToQueue,
                                     BaseType_t * const pxHigherPriorityTaskWoken,
                                     const BaseType_t xCopyPosition )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_err_t err = -RT_ERROR;

    configASSERT( pxQueue );

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );
    if ( type == RT_Object_Class_MessageQueue )
    {
        if ( xCopyPosition == queueSEND_TO_BACK )
        {
            err = rt_mq_send( ( rt_mq_t ) pipc, pvItemToQueue, ( ( rt_mq_t ) pipc )->msg_size);
        }
        else if ( xCopyPosition == queueSEND_TO_FRONT )
        {
            err = rt_mq_urgent( ( rt_mq_t ) pipc, pvItemToQueue, ( ( rt_mq_t ) pipc )->msg_size );
        }
    }

    return rt_err_to_freertos( err );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue,
                              BaseType_t * const pxHigherPriorityTaskWoken )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_base_t level;
    rt_err_t err = -RT_ERROR;

    configASSERT( pxQueue );

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );
    RT_ASSERT( type != RT_Object_Class_Mutex );
    if ( type == RT_Object_Class_Semaphore )
    {
        level = rt_hw_interrupt_disable();
        if ( ( ( rt_sem_t ) pipc )->value < ( ( struct rt_semaphore_wrapper * ) pipc )->max_value )
        {
            err = rt_sem_release( ( rt_sem_t ) pipc );
        }
        rt_hw_interrupt_enable( level );
    }
    if ( pxHigherPriorityTaskWoken != NULL )
    {
        *pxHigherPriorityTaskWoken = pdFALSE;
    }

    return rt_err_to_freertos( err );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueReceive( QueueHandle_t xQueue,
                          void * const pvBuffer,
                          TickType_t xTicksToWait )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_err_t err = -RT_ERROR;

    /* Check the queue pointer is not NULL. */
    configASSERT( ( pxQueue ) );

    /* Cannot block if the scheduler is suspended. */
    #if ( INCLUDE_xTaskGetSchedulerState == 1 )
        {
            configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) && ( xTicksToWait != 0 ) ) );
        }
    #endif

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );
    if ( type == RT_Object_Class_MessageQueue )
    {
        err = ( rt_err_t ) rt_mq_recv( ( rt_mq_t ) pipc, pvBuffer, ( ( rt_mq_t ) pipc )->msg_size, ( rt_int32_t ) xTicksToWait );
#if RT_VER_NUM >= 0x50001
        if (( rt_ssize_t ) err >= 0)
        {
            err = RT_EOK;
        }
#endif
    }

    return rt_err_to_freertos( err );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSemaphoreTake( QueueHandle_t xQueue,
                                TickType_t xTicksToWait )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_err_t err = -RT_ERROR;

    /* Check the queue pointer is not NULL. */
    configASSERT( ( pxQueue ) );

    /* Cannot block if the scheduler is suspended. */
    #if ( INCLUDE_xTaskGetSchedulerState == 1 )
        {
            configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) && ( xTicksToWait != 0 ) ) );
        }
    #endif

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );
    if ( type == RT_Object_Class_Mutex )
    {
        err = rt_mutex_take( ( rt_mutex_t ) pipc, ( rt_int32_t ) xTicksToWait );
    }
    else if ( type == RT_Object_Class_Semaphore )
    {
        err = rt_sem_take( ( rt_sem_t ) pipc, ( rt_int32_t ) xTicksToWait );
    }

    return rt_err_to_freertos( err );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue,
                                 void * const pvBuffer,
                                 BaseType_t * const pxHigherPriorityTaskWoken )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_err_t err = -RT_ERROR;

    configASSERT( pxQueue );

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );
    RT_ASSERT( type != RT_Object_Class_Mutex );
    if ( type == RT_Object_Class_Semaphore )
    {
        err = rt_sem_take( ( rt_sem_t ) pipc, RT_WAITING_NO );
    }
    else if ( type == RT_Object_Class_MessageQueue )
    {
        err = ( rt_err_t ) rt_mq_recv( ( rt_mq_t ) pipc, pvBuffer, ( ( rt_mq_t ) pipc )->msg_size, RT_WAITING_NO );
#if RT_VER_NUM >= 0x50001
        if (( rt_ssize_t ) err >= 0)
        {
            err = RT_EOK;
        }
#endif
    }
    if ( pxHigherPriorityTaskWoken != NULL )
    {
        *pxHigherPriorityTaskWoken = pdFALSE;
    }

    return rt_err_to_freertos( err );
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaiting( const QueueHandle_t xQueue )
{
    UBaseType_t uxReturn = 0;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_base_t level;

    configASSERT( xQueue );

    pipc = xQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );

    level = rt_hw_interrupt_disable();

    if ( type == RT_Object_Class_Mutex )
    {
        if ( ( ( rt_mutex_t ) pipc )->owner == RT_NULL )
        {
            uxReturn = 1;
        }
        else
        {
            uxReturn = 0;
        }
    }
    else if ( type == RT_Object_Class_Semaphore )
    {
        uxReturn = ( ( rt_sem_t ) pipc )->value;
    }
    else if ( type == RT_Object_Class_MessageQueue )
    {
        uxReturn = ( ( rt_mq_t ) pipc )->entry;
    }

    rt_hw_interrupt_enable( level );

    return uxReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueSpacesAvailable( const QueueHandle_t xQueue )
{
    UBaseType_t uxReturn = 0;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_base_t level;

    configASSERT( xQueue );

    pipc = xQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );

    level = rt_hw_interrupt_disable();

    if ( type == RT_Object_Class_Mutex )
    {
        if ( ( ( rt_mutex_t ) pipc )->owner == RT_NULL )
        {
            uxReturn = 0;
        }
        else
        {
            uxReturn = 1;
        }
    }
    else if ( type == RT_Object_Class_Semaphore )
    {
        uxReturn = ( ( struct rt_semaphore_wrapper * ) pipc )->max_value - ( ( rt_sem_t ) pipc )->value;
    }
    else if ( type == RT_Object_Class_MessageQueue )
    {
        uxReturn = ( ( rt_mq_t ) pipc )->max_msgs - ( ( rt_mq_t ) pipc )->entry;
    }

    rt_hw_interrupt_enable( level );

    return uxReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaitingFromISR( const QueueHandle_t xQueue )
{
    return uxQueueMessagesWaiting( xQueue );
}
/*-----------------------------------------------------------*/

void vQueueDelete( QueueHandle_t xQueue )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;

    configASSERT( pxQueue );

    pipc = pxQueue->rt_ipc;
    RT_ASSERT( pipc != RT_NULL );
    type = rt_object_get_type( &pipc->parent );
#if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
    if ( rt_object_is_systemobject( ( rt_object_t ) pipc ) )
#endif
    {
    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        if ( type == RT_Object_Class_Mutex )
        {
            rt_mutex_detach( ( rt_mutex_t ) pipc );
        }
        else if ( type == RT_Object_Class_Semaphore )
        {
            rt_sem_detach( ( rt_sem_t ) pipc );
        }
        else if ( type == RT_Object_Class_MessageQueue )
        {
            rt_mq_detach( ( rt_mq_t ) pipc );
        }
    #endif
#if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
    }
    else
    {
#endif
    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
        if ( type == RT_Object_Class_Mutex )
        {
            rt_mutex_delete( ( rt_mutex_t ) pipc );
        }
        else if ( type == RT_Object_Class_Semaphore )
        {
            /* Allocated with rt_sem_init in xQueueGenericCreate */
            pipc->parent.type |= RT_Object_Class_Static;
            rt_sem_detach( ( rt_sem_t ) pipc );
            RT_KERNEL_FREE( pipc );
        }
        else if ( type == RT_Object_Class_MessageQueue )
        {
            rt_mq_delete( ( rt_mq_t ) pipc );
        }
        else
        {
            return;
        }
        RT_KERNEL_FREE( pxQueue );
    #endif
    }
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueEmptyFromISR( const QueueHandle_t xQueue )
{
    BaseType_t xReturn;

    configASSERT( xQueue );

    if( uxQueueMessagesWaiting( xQueue ) == ( UBaseType_t ) 0 )
    {
        xReturn = pdTRUE;
    }
    else
    {
        xReturn = pdFALSE;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueFullFromISR( const QueueHandle_t xQueue )
{
    BaseType_t xReturn;

    configASSERT( xQueue );

    if ( uxQueueSpacesAvailable( xQueue ) == ( UBaseType_t ) 0 )
    {
        xReturn = pdTRUE;
    }
    else
    {
        xReturn = pdFALSE;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

#ifdef ESP_PLATFORM
/* Unimplemented */
#include "esp_log.h"
static const char *TAG = "freertos";
QueueSetHandle_t xQueueCreateSet( const UBaseType_t uxEventQueueLength )
{
    ESP_LOGE(TAG, "xQueueCreateSet unimplemented");
    configASSERT(0);
    return NULL;
}
BaseType_t xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                           QueueSetHandle_t xQueueSet )
{
    ESP_LOGE(TAG, "xQueueAddToSet unimplemented");
    configASSERT(0);
    return pdFAIL;
}

BaseType_t xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                QueueSetHandle_t xQueueSet )
{
    ESP_LOGE(TAG, "xQueueRemoveFromSet unimplemented");
    configASSERT(0);
    return pdFAIL;
}

QueueSetMemberHandle_t xQueueSelectFromSet( QueueSetHandle_t xQueueSet,
                                            const TickType_t xTicksToWait )
{
    ESP_LOGE(TAG, "xQueueSelectFromSet unimplemented");
    configASSERT(0);
    return NULL;
}

QueueSetMemberHandle_t xQueueSelectFromSetFromISR( QueueSetHandle_t xQueueSet )
{
    ESP_LOGE(TAG, "xQueueSelectFromSetFromISR unimplemented");
    configASSERT(0);
    return NULL;
}

BaseType_t xQueuePeek( QueueHandle_t xQueue,
                       void * const pvBuffer,
                       TickType_t xTicksToWait )
{
    ESP_LOGE(TAG, "xQueuePeek unimplemented");
    configASSERT(0);
    return pdFAIL;
}

BaseType_t xQueueOverwrite(QueueHandle_t xQueue, const void * pvItemToQueue)
{
    ESP_LOGE(TAG, "xQueueOverwrite unimplemented");
    configASSERT(0);
    return pdFAIL;
}

BaseType_t xQueueOverwriteFromISR(QueueHandle_t xQueue, const void * pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken)
{
    ESP_LOGE(TAG, "xQueueOverwriteFromISR unimplemented");
    configASSERT(0);
    return pdFAIL;
}
#endif
