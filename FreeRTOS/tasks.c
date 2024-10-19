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

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Values that can be assigned to the ucNotifyState member of the TCB. */
#define taskNOT_WAITING_NOTIFICATION              ( ( uint8_t ) 0 ) /* Must be zero as it is the initialised value. */
#define taskWAITING_NOTIFICATION                  ( ( uint8_t ) 1 )
#define taskNOTIFICATION_RECEIVED                 ( ( uint8_t ) 2 )

/*
 * Several functions take a TaskHandle_t parameter that can optionally be NULL,
 * where NULL is used to indicate that the handle of the currently executing
 * task should be used in place of the parameter.  This macro simply checks to
 * see if the parameter is NULL and returns a pointer to the appropriate TCB.
 */
#define prvGetTCBFromHandle( pxHandle )    ( ( ( pxHandle ) == NULL ) ? ( xTaskGetCurrentTaskHandle() ) : ( pxHandle ) )

/*
 * Task control block.  A task control block (TCB) is allocated for each task,
 * and stores task state information, including a pointer to the task's context
 * (the task's run time environment, including register values)
 */
typedef struct tskTaskControlBlock
{
    struct rt_thread thread;
    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        TaskHookFunction_t pxTaskTag;
    #endif
    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        volatile uint32_t ulNotifiedValue[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
        volatile uint8_t ucNotifyState[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
    #endif
    #if ( INCLUDE_xTaskAbortDelay == 1 )
        uint8_t ucDelayAborted;
    #endif
} tskTCB;
typedef tskTCB TCB_t;

/* Other file private variables. --------------------------------*/
static volatile BaseType_t xSchedulerRunning = pdFALSE;

/*-----------------------------------------------------------*/

/*
 * Called after a Task_t structure has been allocated either statically or
 * dynamically to fill in the structure's members.
 */
static void prvInitialiseNewTask( TaskFunction_t pxTaskCode,
                                  const char * const pcName,
                                  const uint32_t ulStackDepth,
                                  void * const pvParameters,
                                  UBaseType_t uxPriority,
                                  TaskHandle_t * const pxCreatedTask,
                                  TCB_t * pxNewTCB,
                                  StackType_t * const puxStackBuffer );

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    TaskHandle_t xTaskCreateStatic( TaskFunction_t pxTaskCode,
                                    const char * const pcName,
                                    const uint32_t ulStackDepth,
                                    void * const pvParameters,
                                    UBaseType_t uxPriority,
                                    StackType_t * const puxStackBuffer,
                                    StaticTask_t * const pxTaskBuffer )
    {
        TCB_t * pxNewTCB;
        TaskHandle_t xReturn = NULL;

        configASSERT( puxStackBuffer != NULL );
        configASSERT( pxTaskBuffer != NULL );

        #if ( configASSERT_DEFINED == 1 )
            {
                /* Sanity check that the size of the structure used to declare a
                 * variable of type StaticTask_t equals the size of the real task
                 * structure. */
                volatile size_t xSize = sizeof( StaticTask_t );
                configASSERT( xSize == sizeof( TCB_t ) );
                ( void ) xSize; /* Prevent lint warning when configASSERT() is not used. */
            }
        #endif /* configASSERT_DEFINED */

        if( ( pxTaskBuffer != NULL ) && ( puxStackBuffer != NULL ) )
        {
            pxNewTCB = ( TCB_t * ) pxTaskBuffer;
            prvInitialiseNewTask( pxTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, &xReturn, pxNewTCB, puxStackBuffer );
            rt_thread_startup( ( rt_thread_t ) pxNewTCB );
        }

        return xReturn;
    }

#endif /* SUPPORT_STATIC_ALLOCATION */

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

    BaseType_t xTaskCreate( TaskFunction_t pxTaskCode,
                            const char * const pcName,
                            const configSTACK_DEPTH_TYPE usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pxCreatedTask )
    {
        TCB_t * pxNewTCB;
        BaseType_t xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
        void * stack_start = RT_NULL;

        pxNewTCB = ( TCB_t * ) RT_KERNEL_MALLOC( sizeof( TCB_t ) );
        if ( pxNewTCB != NULL )
        {
            stack_start = RT_KERNEL_MALLOC( usStackDepth * sizeof( StackType_t ) );
            if ( stack_start != RT_NULL )
            {
                prvInitialiseNewTask( pxTaskCode, pcName, ( uint32_t ) usStackDepth, pvParameters, uxPriority, pxCreatedTask, pxNewTCB, ( StackType_t * ) stack_start );
                xReturn = pdPASS;
                /* Mark as dynamic */
#if RT_VER_NUM < 0x50000
                ( ( struct rt_thread * ) pxNewTCB )-> type &= ~RT_Object_Class_Static;
#else
                ( ( struct rt_thread * ) pxNewTCB )-> parent.type &= ~RT_Object_Class_Static;
#endif /* RT_VER_NUM < 0x50000 */
                rt_thread_startup( ( rt_thread_t ) pxNewTCB );
            }
            else
            {
                RT_KERNEL_FREE( pxNewTCB );
            }
        }

        return xReturn;
    }

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

#ifdef ESP_PLATFORM
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

    BaseType_t xTaskCreatePinnedToCore( TaskFunction_t pvTaskCode,
                            const char * const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                            const uint32_t usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pvCreatedTask,
                            const BaseType_t xCoreID)
    {
        ( void ) xCoreID;
        return xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pvCreatedTask );
    }

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
#endif
/*-----------------------------------------------------------*/

static void prvInitialiseNewTask( TaskFunction_t pxTaskCode,
                                  const char * const pcName,
                                  const uint32_t ulStackDepth,
                                  void * const pvParameters,
                                  UBaseType_t uxPriority,
                                  TaskHandle_t * const pxCreatedTask,
                                  TCB_t * pxNewTCB,
                                  StackType_t * const puxStackBuffer )
{
    /* This is used as an array index so must ensure it's not too large. */
    configASSERT( uxPriority < configMAX_PRIORITIES );

    if( uxPriority >= ( UBaseType_t ) configMAX_PRIORITIES )
    {
        uxPriority = ( UBaseType_t ) configMAX_PRIORITIES - ( UBaseType_t ) 1U;
    }

    rt_thread_init( ( struct rt_thread * ) pxNewTCB, pcName, pxTaskCode, pvParameters,
                    puxStackBuffer, ulStackDepth * sizeof( StackType_t ), FREERTOS_PRIORITY_TO_RTTHREAD( uxPriority ), 1 );

#if ( configUSE_APPLICATION_TASK_TAG == 1 )
    pxNewTCB->pxTaskTag = NULL;
#endif

#if ( configUSE_TASK_NOTIFICATIONS == 1 )
    rt_memset( ( void * ) &( pxNewTCB->ulNotifiedValue[ 0 ] ), 0x00, sizeof( pxNewTCB->ulNotifiedValue ) );
    rt_memset( ( void * ) &( pxNewTCB->ucNotifyState[ 0 ] ), 0x00, sizeof( pxNewTCB->ucNotifyState ) );
#endif

#if ( INCLUDE_xTaskAbortDelay == 1 )
    pxNewTCB->ucDelayAborted = pdFALSE;
#endif

    if ( pxCreatedTask != NULL )
    {
        *pxCreatedTask = ( TaskHandle_t ) pxNewTCB;
    }
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelete == 1 )

    void vTaskDelete( TaskHandle_t xTaskToDelete )
    {
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTaskToDelete );
        if ( thread == RT_NULL )
        {
            thread = rt_thread_self();
        }
    #if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
        if ( rt_object_is_systemobject( ( rt_object_t ) thread ) )
    #endif
        {
        #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
            rt_thread_detach( thread );
        #endif
    #if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
        }
        else
        {
    #endif
        #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
            rt_thread_delete( thread );
        #endif
        }

        if ( thread == rt_thread_self() )
        {
            rt_schedule();
        }
    }

#endif /* INCLUDE_vTaskDelete */
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskDelayUntil == 1 )

    BaseType_t xTaskDelayUntil( TickType_t * const pxPreviousWakeTime,
                                const TickType_t xTimeIncrement )
    {
        BaseType_t xShouldDelay = pdFALSE;
        rt_base_t level;
        rt_tick_t cur_tick;

        RT_ASSERT( pxPreviousWakeTime != RT_NULL );
        RT_ASSERT( xTimeIncrement > 0U );

        level = rt_hw_interrupt_disable();
        cur_tick = rt_tick_get();
        if (cur_tick - *pxPreviousWakeTime < xTimeIncrement)
        {
            rt_thread_delay_until( pxPreviousWakeTime, xTimeIncrement );
            xShouldDelay = pdTRUE;
        }
        rt_hw_interrupt_enable( level );

        return xShouldDelay;
    }

#endif /* INCLUDE_xTaskDelayUntil */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelay == 1 )

    void vTaskDelay( const TickType_t xTicksToDelay )
    {
        rt_thread_delay( xTicksToDelay );
    }

#endif /* INCLUDE_vTaskDelay */
/*-----------------------------------------------------------*/

#if ( ( INCLUDE_eTaskGetState == 1 ) || ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_xTaskAbortDelay == 1 ) )

    eTaskState eTaskGetState( TaskHandle_t xTask )
    {
        eTaskState eReturn;
        rt_thread_t thread = ( rt_thread_t ) xTask;
        rt_base_t level;

        configASSERT( xTask );

        level = rt_hw_interrupt_disable();
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION < RT_VERSION_CHECK(5, 2, 0))
        switch ( thread->stat & RT_THREAD_STAT_MASK )
#else
        switch ( RT_SCHED_CTX(thread).stat & RT_THREAD_STAT_MASK )
#endif
        {
            case RT_THREAD_READY:
            {
                eReturn = eReady;
                break;
            }
            case RT_THREAD_SUSPEND:
            {
                /* If thread timer is activated it is blocked with a timeout */
                if ( thread->thread_timer.parent.flag & RT_TIMER_FLAG_ACTIVATED )
                {
                    eReturn = eBlocked;
                }
                /* Otherwise it is suspended or blocked with an infinite timeout */
                else
                {
                    eReturn = eSuspended;
                }
                break;
            }
            case RT_THREAD_RUNNING:
            {
                eReturn = eRunning;
                break;
            }
            case RT_THREAD_CLOSE:
            {
                eReturn = eDeleted;
                break;
            }
            default:
                eReturn = eInvalid;
        }

        rt_hw_interrupt_enable( level );

        return eReturn;
    }

#endif /* INCLUDE_eTaskGetState */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )

    UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask )
    {
        UBaseType_t uxReturn;
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTask );
        rt_base_t level;

        level = rt_hw_interrupt_disable();
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION < RT_VERSION_CHECK(5, 2, 0))
        uxReturn = thread->current_priority;
#else
        uxReturn = RT_SCHED_PRIV(thread).current_priority;
#endif
        rt_hw_interrupt_enable( level );

        return RTTHREAD_PRIORITY_TO_FREERTOS( uxReturn );
    }

#endif /* INCLUDE_uxTaskPriorityGet */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )

    UBaseType_t uxTaskPriorityGetFromISR( const TaskHandle_t xTask )
    {
        return uxTaskPriorityGet( xTask );
    }

#endif /* INCLUDE_uxTaskPriorityGet */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskPrioritySet == 1 )

    void vTaskPrioritySet( TaskHandle_t xTask,
                           UBaseType_t uxNewPriority )
    {
        extern rt_thread_t rt_current_thread;
        rt_thread_t thread;
        rt_uint8_t current_priority;
        rt_bool_t need_schedule = RT_FALSE;
        rt_base_t level;

        configASSERT( uxNewPriority < configMAX_PRIORITIES );

        /* Ensure the new priority is valid. */
        if( uxNewPriority >= ( UBaseType_t ) configMAX_PRIORITIES )
        {
            uxNewPriority = ( UBaseType_t ) configMAX_PRIORITIES - ( UBaseType_t ) 1U;
        }
        uxNewPriority = FREERTOS_PRIORITY_TO_RTTHREAD( uxNewPriority );

        level = rt_hw_interrupt_disable();

        thread = ( rt_thread_t ) prvGetTCBFromHandle( xTask );
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION < RT_VERSION_CHECK(5, 2, 0))
        current_priority = thread->current_priority;
#else
        current_priority = RT_SCHED_PRIV(thread).current_priority;
#endif
        if ( current_priority != uxNewPriority )
        {
            rt_thread_control( thread, RT_THREAD_CTRL_CHANGE_PRIORITY, &uxNewPriority);
            if ( uxNewPriority < current_priority )
            {
                /* The priority of a task other than the currently running task is being raised.
                 * Need to schedule if the priority is raised above that of the running task */
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION < RT_VERSION_CHECK(5, 2, 0))
                if ( thread != rt_current_thread && uxNewPriority <= rt_current_thread->current_priority )
#else
                if ( thread != rt_current_thread && uxNewPriority <= RT_SCHED_PRIV(rt_current_thread).current_priority )
#endif
                {
                    need_schedule = RT_TRUE;
                }
            }
            /* Setting the priority of the running task down means
             * there may now be another task of higher priority that
             * is ready to execute. */
            else if ( thread == rt_current_thread )
            {
                need_schedule = RT_TRUE;
            }
        }

        rt_hw_interrupt_enable( level );

        if ( need_schedule == RT_TRUE )
        {
            rt_schedule();
        }
    }

#endif /* INCLUDE_vTaskPrioritySet */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

    void vTaskSuspend( TaskHandle_t xTaskToSuspend )
    {
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTaskToSuspend );
        if ( rt_thread_suspend( thread ) == RT_EOK )
        {
            rt_schedule();
        }
    }

#endif /* INCLUDE_vTaskSuspend */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

    void vTaskResume( TaskHandle_t xTaskToResume )
    {
        rt_thread_t thread = ( rt_thread_t ) xTaskToResume;
        rt_bool_t need_schedule = RT_FALSE;
        rt_base_t level;

        /* It does not make sense to resume the calling task. */
        configASSERT( xTaskToResume );

        if ( thread != NULL && thread != rt_thread_self() )
        {
            level = rt_hw_interrupt_disable();
            /* A task with higher priority than the current running task is ready */
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION < RT_VERSION_CHECK(5, 2, 0))
            if ( rt_thread_resume( thread ) == RT_EOK && thread->current_priority <= rt_thread_self()->current_priority )
#else
            if ( rt_thread_resume( thread ) == RT_EOK && RT_SCHED_PRIV(thread).current_priority <= RT_SCHED_PRIV(rt_thread_self()).current_priority )
#endif
            {
                need_schedule = RT_TRUE;
            }
            rt_hw_interrupt_enable( level );
        }
        if (need_schedule == RT_TRUE)
        {
            rt_schedule();
        }
    }

#endif /* INCLUDE_vTaskSuspend */

/*-----------------------------------------------------------*/

#if ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) )

    BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume )
    {
        vTaskResume( xTaskToResume );
        return pdFALSE;
    }

#endif /* ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) ) */
/*-----------------------------------------------------------*/

void vTaskStartScheduler( void )
{
    xSchedulerRunning = pdTRUE;
#ifdef ESP_PLATFORM
    extern int rtthread_startup(void);
    rtthread_startup();
#endif
}
/*-----------------------------------------------------------*/

void vTaskEndScheduler( void )
{
    xSchedulerRunning = pdFALSE;
    vPortEndScheduler();
}
/*----------------------------------------------------------*/

void vTaskSuspendAll( void )
{
    rt_enter_critical();
}
/*----------------------------------------------------------*/

BaseType_t xTaskResumeAll( void )
{
    rt_exit_critical();
    return pdFALSE;
}
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCount( void )
{
    return rt_tick_get();
}
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCountFromISR( void )
{
    return rt_tick_get();
}
/*-----------------------------------------------------------*/

UBaseType_t uxTaskGetNumberOfTasks( void )
{
    UBaseType_t uxReturn = 0;
    rt_base_t level;
    struct rt_object_information *information;
    struct rt_list_node *node = RT_NULL;

    information = rt_object_get_information( RT_Object_Class_Thread );
    RT_ASSERT( information != RT_NULL );

    level = rt_hw_interrupt_disable();

    rt_list_for_each( node, &( information->object_list ) )
    {
        uxReturn += 1;
    }

    rt_hw_interrupt_enable( level );

    return uxReturn;
}
/*-----------------------------------------------------------*/

char * pcTaskGetName( TaskHandle_t xTaskToQuery )
{
    rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTaskToQuery );
#if RT_VER_NUM < 0x50000
    return &( thread->name[ 0 ] );
#else
    return &( thread->parent.name[ 0 ] );
#endif /* RT_VER_NUM < 0x50000 */
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetHandle == 1 )

    TaskHandle_t xTaskGetHandle( const char * pcNameToQuery )
    {
        return ( TaskHandle_t ) rt_thread_find( ( char * ) pcNameToQuery );
    }

#endif /* INCLUDE_xTaskGetHandle */
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )

    TaskHandle_t xTaskGetIdleTaskHandle( void )
    {
        return ( TaskHandle_t ) rt_thread_find( "tidle0" );
    }

#endif /* INCLUDE_xTaskGetIdleTaskHandle */
/*----------------------------------------------------------*/

#if ( INCLUDE_xTaskAbortDelay == 1 )

    BaseType_t xTaskAbortDelay( TaskHandle_t xTask )
    {
        TCB_t * pxTCB = xTask;
        BaseType_t xReturn;
        rt_thread_t thread = ( rt_thread_t ) xTask;
        rt_bool_t need_schedule = RT_FALSE;
        rt_base_t level;

        configASSERT( pxTCB );

        level = rt_hw_interrupt_disable();

        if ( eTaskGetState( xTask ) == eBlocked )
        {
            rt_thread_resume( thread );
            thread->error = -RT_ETIMEOUT;
            pxTCB->ucDelayAborted = pdTRUE;
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION < RT_VERSION_CHECK(5, 2, 0))
            if ( thread->current_priority < rt_thread_self()->current_priority )
#else
            if ( RT_SCHED_PRIV(thread).current_priority < RT_SCHED_PRIV(rt_thread_self()).current_priority )
#endif
            {
                need_schedule = RT_TRUE;
            }
            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
        }

        rt_hw_interrupt_enable( level );

        if ( need_schedule == RT_TRUE )
        {
            rt_schedule();
        }

        return xReturn;
    }

#endif /* INCLUDE_xTaskAbortDelay */
/*----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

    void vTaskSetApplicationTaskTag( TaskHandle_t xTask,
                                     TaskHookFunction_t pxHookFunction )
    {
        TCB_t * xTCB = prvGetTCBFromHandle( xTask );
        rt_base_t level;

        level = rt_hw_interrupt_disable();
        xTCB->pxTaskTag = pxHookFunction;
        rt_hw_interrupt_enable( level );
    }

#endif /* configUSE_APPLICATION_TASK_TAG */
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

    TaskHookFunction_t xTaskGetApplicationTaskTag( TaskHandle_t xTask )
    {
        TaskHookFunction_t xReturn;
        TCB_t * xTCB = prvGetTCBFromHandle( xTask );
        rt_base_t level;

        level = rt_hw_interrupt_disable();
        xReturn = xTCB->pxTaskTag;
        rt_hw_interrupt_enable( level );

        return xReturn;
    }

#endif /* configUSE_APPLICATION_TASK_TAG */
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

    TaskHookFunction_t xTaskGetApplicationTaskTagFromISR( TaskHandle_t xTask )
    {
        return xTaskGetApplicationTaskTag( xTask );
    }

#endif /* configUSE_APPLICATION_TASK_TAG */
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

    BaseType_t xTaskCallApplicationTaskHook( TaskHandle_t xTask,
                                             void * pvParameter )
    {
        BaseType_t xReturn;
        TCB_t * xTCB = prvGetTCBFromHandle( xTask );

        if( xTCB->pxTaskTag != NULL )
        {
            xReturn = xTCB->pxTaskTag( pvParameter );
        }
        else
        {
            xReturn = pdFAIL;
        }

        return xReturn;
    }

#endif /* configUSE_APPLICATION_TASK_TAG */
/*-----------------------------------------------------------*/

void vTaskSetTimeOutState( TimeOut_t * const pxTimeOut )
{
    rt_base_t level;

    configASSERT( pxTimeOut );
    level = rt_hw_interrupt_disable();
    pxTimeOut->xOverflowCount = 0;
    pxTimeOut->xTimeOnEntering = ( TickType_t ) rt_tick_get();
    rt_hw_interrupt_enable( level );
}
/*-----------------------------------------------------------*/

void vTaskInternalSetTimeOutState( TimeOut_t * const pxTimeOut )
{
    /* For internal use only as it does not use a critical section. */
    pxTimeOut->xOverflowCount = 0;
    pxTimeOut->xTimeOnEntering = ( TickType_t ) rt_tick_get();;
}
/*-----------------------------------------------------------*/

BaseType_t xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut,
                                 TickType_t * const pxTicksToWait )
{
    TCB_t * pxCurrentTCB = ( TCB_t * ) rt_thread_self();
    BaseType_t xReturn;
    rt_base_t level;

    configASSERT( pxTimeOut );
    configASSERT( pxTicksToWait );

    level = rt_hw_interrupt_disable();
    /* Minor optimisation.  The tick count cannot change in this block. */
    const TickType_t xConstTickCount = ( TickType_t ) rt_tick_get();
    const TickType_t xElapsedTime = xConstTickCount - pxTimeOut->xTimeOnEntering;

#if ( INCLUDE_xTaskAbortDelay == 1 )
    if( pxCurrentTCB->ucDelayAborted != ( uint8_t ) pdFALSE )
    {
        /* The delay was aborted, which is not the same as a time out,
         * but has the same result. */
        pxCurrentTCB->ucDelayAborted = pdFALSE;
        xReturn = pdTRUE;
    }
    else
#endif

#if ( INCLUDE_vTaskSuspend == 1 )
    if( *pxTicksToWait == portMAX_DELAY )
    {
        /* If INCLUDE_vTaskSuspend is set to 1 and the block time
         * specified is the maximum block time then the task should block
         * indefinitely, and therefore never time out. */
        xReturn = pdFALSE;
    }
    else
#endif

    if( xElapsedTime < *pxTicksToWait )
    {
        /* Not a genuine timeout. Adjust parameters for time remaining. */
        *pxTicksToWait -= xElapsedTime;
        vTaskInternalSetTimeOutState( pxTimeOut );
        xReturn = pdFALSE;
    }
    else
    {
        *pxTicksToWait = ( TickType_t ) 0;
        xReturn = pdTRUE;
    }
    rt_hw_interrupt_enable( level );

    return xReturn;
}
/*-----------------------------------------------------------*/

#if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) )

    TaskHandle_t xTaskGetCurrentTaskHandle( void )
    {
        TaskHandle_t xReturn;

        /* A critical section is not required as this is not called from
         * an interrupt and the current TCB will always be the same for any
         * individual execution thread. */
        xReturn = ( TaskHandle_t ) rt_thread_self();

        return xReturn;
    }

#endif /* ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) )

    BaseType_t xTaskGetSchedulerState( void )
    {
        BaseType_t xReturn;

        if( xSchedulerRunning == pdFALSE )
        {
            xReturn = taskSCHEDULER_NOT_STARTED;
        }
        else
        {
            if( rt_critical_level() == 0 )
            {
                xReturn = taskSCHEDULER_RUNNING;
            }
            else
            {
                xReturn = taskSCHEDULER_SUSPENDED;
            }
        }

        return xReturn;
    }

#endif /* ( ( INCLUDE_xTaskGetSchedulerState == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    uint32_t ulTaskGenericNotifyTake( UBaseType_t uxIndexToWait,
                                      BaseType_t xClearCountOnExit,
                                      TickType_t xTicksToWait )
    {
        uint32_t ulReturn;
        TCB_t * pxCurrentTCB = ( TCB_t * ) rt_thread_self();
        rt_thread_t thread = ( rt_thread_t ) pxCurrentTCB;
        rt_base_t level;

        configASSERT( uxIndexToWait < configTASK_NOTIFICATION_ARRAY_ENTRIES );

        level = rt_hw_interrupt_disable();
        /* Only block if the notification count is not already non-zero. */
        if( pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] == 0UL )
        {
            /* Mark this task as waiting for a notification. */
            pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskWAITING_NOTIFICATION;

            if( xTicksToWait > ( TickType_t ) 0 )
            {
                rt_thread_suspend( thread );
                if ( ( rt_int32_t ) xTicksToWait > 0 )
                {
                    rt_timer_control(&(thread->thread_timer),
                                     RT_TIMER_CTRL_SET_TIME,
                                     &xTicksToWait);
                    rt_timer_start(&(thread->thread_timer));
                }
                rt_hw_interrupt_enable(level);
                rt_schedule();
                /* Clear thread error. */
                thread->error = RT_EOK;
            }
        }
        rt_hw_interrupt_enable( level );

        level = rt_hw_interrupt_disable();
        ulReturn = pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ];

        if( ulReturn != 0UL )
        {
            if( xClearCountOnExit != pdFALSE )
            {
                pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] = 0UL;
            }
            else
            {
                pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] = ulReturn - ( uint32_t ) 1;
            }
        }

        pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskNOT_WAITING_NOTIFICATION;
        rt_hw_interrupt_enable( level );

        return ulReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t xTaskGenericNotifyWait( UBaseType_t uxIndexToWait,
                                       uint32_t ulBitsToClearOnEntry,
                                       uint32_t ulBitsToClearOnExit,
                                       uint32_t * pulNotificationValue,
                                       TickType_t xTicksToWait )
    {
        BaseType_t xReturn;
        TCB_t * pxCurrentTCB = ( TCB_t * ) rt_thread_self();
        rt_thread_t thread = ( rt_thread_t ) pxCurrentTCB;
        rt_base_t level;

        configASSERT( uxIndexToWait < configTASK_NOTIFICATION_ARRAY_ENTRIES );

        level = rt_hw_interrupt_disable();
        /* Only block if a notification is not already pending. */
        if( pxCurrentTCB->ucNotifyState[ uxIndexToWait ] != taskNOTIFICATION_RECEIVED )
        {
            /* Clear bits in the task's notification value as bits may get
             * set  by the notifying task or interrupt.  This can be used to
             * clear the value to zero. */
            pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] &= ~ulBitsToClearOnEntry;

            /* Mark this task as waiting for a notification. */
            pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskWAITING_NOTIFICATION;

            if( xTicksToWait > ( TickType_t ) 0 )
            {
                rt_thread_suspend( thread );
                if ( ( rt_int32_t ) xTicksToWait > 0 )
                {
                    rt_timer_control(&(thread->thread_timer),
                                     RT_TIMER_CTRL_SET_TIME,
                                     &xTicksToWait);
                    rt_timer_start(&(thread->thread_timer));
                }
                rt_hw_interrupt_enable(level);
                rt_schedule();
                /* Clear thread error. It is not used to determine the function return value. */
                thread->error = RT_EOK;
            }
            else
            {
                rt_hw_interrupt_enable( level );
            }
        }
        else
        {
            rt_hw_interrupt_enable( level );
        }

        level = rt_hw_interrupt_disable();

        if( pulNotificationValue != NULL )
        {
            /* Output the current notification value, which may or may not
             * have changed. */
            *pulNotificationValue = pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ];
        }

        /* If ucNotifyValue is set then either the task never entered the
         * blocked state (because a notification was already pending) or the
         * task unblocked because of a notification.  Otherwise the task
         * unblocked because of a timeout. */
        if( pxCurrentTCB->ucNotifyState[ uxIndexToWait ] != taskNOTIFICATION_RECEIVED )
        {
            /* A notification was not received. */
            xReturn = pdFALSE;
        }
        else
        {
            /* A notification was already pending or a notification was
             * received while the task was waiting. */
            pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] &= ~ulBitsToClearOnExit;
            xReturn = pdTRUE;
        }

        pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskNOT_WAITING_NOTIFICATION;
        rt_hw_interrupt_enable( level );

        return xReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t xTaskGenericNotify( TaskHandle_t xTaskToNotify,
                                   UBaseType_t uxIndexToNotify,
                                   uint32_t ulValue,
                                   eNotifyAction eAction,
                                   uint32_t * pulPreviousNotificationValue )
    {
        TCB_t * pxTCB;
        BaseType_t xReturn = pdPASS;
        uint8_t ucOriginalNotifyState;
        rt_base_t level;

        configASSERT( uxIndexToNotify < configTASK_NOTIFICATION_ARRAY_ENTRIES );
        configASSERT( xTaskToNotify );
        pxTCB = xTaskToNotify;

        level = rt_hw_interrupt_disable();

        if( pulPreviousNotificationValue != NULL )
        {
            *pulPreviousNotificationValue = pxTCB->ulNotifiedValue[ uxIndexToNotify ];
        }

        ucOriginalNotifyState = pxTCB->ucNotifyState[ uxIndexToNotify ];

        pxTCB->ucNotifyState[ uxIndexToNotify ] = taskNOTIFICATION_RECEIVED;

        switch( eAction )
        {
            case eSetBits:
                pxTCB->ulNotifiedValue[ uxIndexToNotify ] |= ulValue;
                break;

            case eIncrement:
                ( pxTCB->ulNotifiedValue[ uxIndexToNotify ] )++;
                break;

            case eSetValueWithOverwrite:
                pxTCB->ulNotifiedValue[ uxIndexToNotify ] = ulValue;
                break;

            case eSetValueWithoutOverwrite:

                if( ucOriginalNotifyState != taskNOTIFICATION_RECEIVED )
                {
                    pxTCB->ulNotifiedValue[ uxIndexToNotify ] = ulValue;
                }
                else
                {
                    /* The value could not be written to the task. */
                    xReturn = pdFAIL;
                }

                break;

            case eNoAction:

                /* The task is being notified without its notify value being
                 * updated. */
                break;

            default:

                /* Should not get here if all enums are handled.
                 * Artificially force an assert by testing a value the
                 * compiler can't assume is const. */
                configASSERT( xTaskToNotify == NULL );

                break;
        }


        /* If the task is in the blocked state specifically to wait for a
         * notification then unblock it now. */
        if( ucOriginalNotifyState == taskWAITING_NOTIFICATION )
        {
            rt_thread_resume( ( rt_thread_t ) pxTCB );
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION < RT_VERSION_CHECK(5, 2, 0))
            if( ( ( rt_thread_t ) pxTCB )->current_priority < rt_thread_self()->current_priority )
#else
            if( RT_SCHED_PRIV( ( rt_thread_t ) pxTCB ).current_priority < RT_SCHED_PRIV( rt_thread_self() ).current_priority )
#endif
            {
                /* The notified task has a priority above the currently
                 * executing task so a schedule is required. */
                rt_schedule();
            }
        }
        rt_hw_interrupt_enable( level );

        return xReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t xTaskGenericNotifyFromISR( TaskHandle_t xTaskToNotify,
                                          UBaseType_t uxIndexToNotify,
                                          uint32_t ulValue,
                                          eNotifyAction eAction,
                                          uint32_t * pulPreviousNotificationValue,
                                          BaseType_t * pxHigherPriorityTaskWoken )
    {
        BaseType_t xReturn;

        xReturn = xTaskGenericNotify( xTaskToNotify, uxIndexToNotify, ulValue, eAction, pulPreviousNotificationValue );
        if ( pxHigherPriorityTaskWoken != NULL )
        {
            *pxHigherPriorityTaskWoken = pdFALSE;
        }

        return xReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    void vTaskGenericNotifyGiveFromISR( TaskHandle_t xTaskToNotify,
                                        UBaseType_t uxIndexToNotify,
                                        BaseType_t * pxHigherPriorityTaskWoken )
    {
        xTaskNotifyGiveIndexed( xTaskToNotify, uxIndexToNotify );
        if ( pxHigherPriorityTaskWoken != NULL )
        {
            *pxHigherPriorityTaskWoken = pdFALSE;
        }
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t xTaskGenericNotifyStateClear( TaskHandle_t xTask,
                                             UBaseType_t uxIndexToClear )
    {
        TCB_t * pxTCB;
        BaseType_t xReturn;
        rt_base_t level;

        configASSERT( uxIndexToClear < configTASK_NOTIFICATION_ARRAY_ENTRIES );

        /* If null is passed in here then it is the calling task that is having
         * its notification state cleared. */
        pxTCB = prvGetTCBFromHandle( xTask );

        level = rt_hw_interrupt_disable();

        if( pxTCB->ucNotifyState[ uxIndexToClear ] == taskNOTIFICATION_RECEIVED )
        {
            pxTCB->ucNotifyState[ uxIndexToClear ] = taskNOT_WAITING_NOTIFICATION;
            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
        }

        rt_hw_interrupt_enable( level );

        return xReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    uint32_t ulTaskGenericNotifyValueClear( TaskHandle_t xTask,
                                            UBaseType_t uxIndexToClear,
                                            uint32_t ulBitsToClear )
    {
        TCB_t * pxTCB;
        uint32_t ulReturn;
        rt_base_t level;

        /* If null is passed in here then it is the calling task that is having
         * its notification state cleared. */
        pxTCB = prvGetTCBFromHandle( xTask );

        level = rt_hw_interrupt_disable();

        /* Return the notification as it was before the bits were cleared,
         * then clear the bit mask. */
        ulReturn = pxTCB->ulNotifiedValue[ uxIndexToClear ];
        pxTCB->ulNotifiedValue[ uxIndexToClear ] &= ~ulBitsToClear;

        rt_hw_interrupt_enable( level );

        return ulReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark2 == 1 )

/* uxTaskGetStackHighWaterMark() and uxTaskGetStackHighWaterMark2() are the
 * same except for their return type.  Using configSTACK_DEPTH_TYPE allows the
 * user to determine the return type.  It gets around the problem of the value
 * overflowing on 8-bit types without breaking backward compatibility for
 * applications that expect an 8-bit return type. */
    configSTACK_DEPTH_TYPE uxTaskGetStackHighWaterMark2( TaskHandle_t xTask )
    {
        uint32_t ulCount = 0U;
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTask );
        rt_uint8_t * stack_addr = thread->stack_addr;

    #ifdef ARCH_CPU_STACK_GROWS_UPWARD
        stack_addr = stack_addr + thread->stack_size - 1;
        while ( *stack_addr == '#' )
        {
            ulCount += 1;
            stack_addr -= 1;
        }
    #else
        while ( *stack_addr == '#' )
        {
            ulCount += 1;
            stack_addr += 1;
        }
    #endif

        ulCount /= ( uint32_t ) sizeof( StackType_t );

        return ( configSTACK_DEPTH_TYPE ) ulCount;
    }

#endif /* INCLUDE_uxTaskGetStackHighWaterMark2 */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )

    UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask )
    {
        return ( UBaseType_t ) uxTaskGetStackHighWaterMark2( xTask );
    }

#endif /* INCLUDE_uxTaskGetStackHighWaterMark */
/*-----------------------------------------------------------*/


#ifdef ESP_PLATFORM
BaseType_t xTaskGetAffinity( TaskHandle_t xTask )
{
    ( void ) xTask;
    return 0;
}

TaskHandle_t xTaskGetCurrentTaskHandleForCPU( BaseType_t cpuid )
{
    ( void ) cpuid;
    return xTaskGetCurrentTaskHandle();
}

TaskHandle_t xTaskGetIdleTaskHandleForCPU( UBaseType_t cpuid )
{
    ( void ) cpuid;
    return xTaskGetIdleTaskHandle();
}

#if ( configINCLUDE_FREERTOS_TASK_C_ADDITIONS_H == 1 )

    #include "freertos_tasks_c_additions.h"

    #ifdef FREERTOS_TASKS_C_ADDITIONS_INIT
        static void freertos_tasks_c_additions_init( void )
        {
            FREERTOS_TASKS_C_ADDITIONS_INIT();
        }
    #endif

#endif /* if ( configINCLUDE_FREERTOS_TASK_C_ADDITIONS_H == 1 ) */

/* Unimplemented */
#include "esp_log.h"
static const char *TAG = "freertos";
#if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
void vTaskSetThreadLocalStoragePointer( TaskHandle_t xTaskToSet,
                                            BaseType_t xIndex,
                                            void * pvValue )
{
    ESP_LOGE(TAG, "vTaskSetThreadLocalStoragePointer unimplemented");
    configASSERT(0);
}
void * pvTaskGetThreadLocalStoragePointer( TaskHandle_t xTaskToQuery,
                                               BaseType_t xIndex )
{
    ESP_LOGE(TAG, "pvTaskGetThreadLocalStoragePointer unimplemented");
    configASSERT(0);
    return NULL;
}
#if ( configTHREAD_LOCAL_STORAGE_DELETE_CALLBACKS )
typedef void (*TlsDeleteCallbackFunction_t)( int, void * );
void vTaskSetThreadLocalStoragePointerAndDelCallback( TaskHandle_t xTaskToSet, BaseType_t xIndex, void *pvValue, TlsDeleteCallbackFunction_t pvDelCallback)
{
    ESP_LOGE(TAG, "vTaskSetThreadLocalStoragePointerAndDelCallback unimplemented");
    configASSERT(0);
}
#endif
#endif
#endif
