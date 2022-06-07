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

        if( ( uxQueueLength > ( UBaseType_t ) 0 ) &&
            ( pxStaticQueue != NULL ) &&

            /* A queue storage area should be provided if the item size is not 0, and
             * should not be provided if the item size is 0. */
            ( !( ( pucQueueStorage != NULL ) && ( uxItemSize == 0 ) ) ) &&
            ( !( ( pucQueueStorage == NULL ) && ( uxItemSize != 0 ) ) ) )
        {
            if ( ( ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX ) )
            {
                rt_snprintf( name, RT_NAME_MAX - 1, "mutex%02d", mutex_index++ );
                rt_mutex_init( ( rt_mutex_t ) &( ( StaticSemaphore_t * ) pxStaticQueue )->ipc_obj.mutex, name, RT_IPC_FLAG_PRIO );
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
            if ( ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX )
            {
                rt_snprintf( name, RT_NAME_MAX - 1, "mutex%02d", mutex_index++ );
                pipc = ( struct rt_ipc_object * ) rt_mutex_create( name, RT_IPC_FLAG_PRIO );
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

BaseType_t xQueueGenericSend( QueueHandle_t xQueue,
                              const void * const pvItemToQueue,
                              TickType_t xTicksToWait,
                              const BaseType_t xCopyPosition )
{
    Queue_t * const pxQueue = xQueue;
    struct rt_ipc_object *pipc;
    rt_uint8_t type;
    rt_err_t err = -RT_ERROR;

    configASSERT( pxQueue );
    configASSERT( !( ( pvItemToQueue == NULL ) && ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );
    configASSERT( !( ( xCopyPosition == queueOVERWRITE ) && ( pxQueue->uxLength != 1 ) ) );
    #if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
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
    #if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
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

    return rt_err_to_freertos( err );
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
        else if ( type == RT_Object_Class_MailBox )
        {
            rt_mb_detach( ( rt_mailbox_t ) pipc );
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
            rt_sem_delete( ( rt_sem_t ) pipc );
        }
        else if ( type == RT_Object_Class_MailBox )
        {
            rt_mb_delete( ( rt_mailbox_t ) pipc );
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
