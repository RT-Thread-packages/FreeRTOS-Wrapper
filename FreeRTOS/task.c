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
} tskTCB;
typedef tskTCB TCB_t;

/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    TaskHandle_t xTaskCreateStatic( TaskFunction_t pxTaskCode,
                                    const char * const pcName,
                                    const uint32_t ulStackDepth,
                                    void * const pvParameters,
                                    UBaseType_t uxPriority,
                                    StackType_t * const puxStackBuffer,
                                    StaticTask_t * const pxTaskBuffer )
    {
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
            rt_thread_init( ( struct rt_thread * ) pxTaskBuffer, pcName, pxTaskCode, pvParameters,
                            ( void * ) puxStackBuffer, ulStackDepth * sizeof( StackType_t ), uxPriority, 1 );
            rt_thread_startup( ( rt_thread_t ) pxTaskBuffer );
            xReturn = ( TaskHandle_t ) pxTaskBuffer;
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
        BaseType_t xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
        rt_thread_t thread = RT_NULL;

        thread = rt_thread_create( pcName, pxTaskCode, pvParameters, usStackDepth * sizeof( StackType_t ), uxPriority, 1 );
        if ( thread != RT_NULL )
        {
            rt_thread_startup( thread );
            xReturn = pdPASS;
        }
        if ( pxCreatedTask != NULL )
        {
            *pxCreatedTask = ( TaskHandle_t ) thread;
        }

        return xReturn;
    }

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelete == 1 )

    void vTaskDelete( TaskHandle_t xTaskToDelete )
    {
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTaskToDelete );
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

        configASSERT( pxTCB );

        level = rt_hw_interrupt_disable();

        switch ( thread->stat & RT_THREAD_STAT_MASK )
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
        uxReturn = thread->current_priority;
        rt_hw_interrupt_enable( level );

        return uxReturn;
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

        level = rt_hw_interrupt_disable();

        thread = ( rt_thread_t ) prvGetTCBFromHandle( xTask );
        current_priority = thread->current_priority;
        if ( current_priority != uxNewPriority )
        {
            rt_thread_control( thread, RT_THREAD_CTRL_CHANGE_PRIORITY, &uxNewPriority);
            if ( uxNewPriority < current_priority )
            {
                /* The priority of a task other than the currently running task is being raised.
                 * Need to schedule if the priority is raised above that of the running task */
                if ( thread != rt_current_thread && uxNewPriority <= rt_current_thread->current_priority )
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
            if ( rt_thread_resume( thread ) == RT_EOK && thread->current_priority <= rt_thread_self()->current_priority )
            {
                need_schedule = RT_TRUE;
            }
            rt_hw_interrupt_enable(level);
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
    return &( thread->name[ 0 ] );
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
            if ( thread->current_priority < rt_thread_self()->current_priority ){
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
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTask );
        rt_base_t level;

        level = rt_hw_interrupt_disable();
        thread->user_data = ( rt_ubase_t ) pxHookFunction;
        rt_hw_interrupt_enable( level );
    }

#endif /* configUSE_APPLICATION_TASK_TAG */
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

    TaskHookFunction_t xTaskGetApplicationTaskTag( TaskHandle_t xTask )
    {
        TaskHookFunction_t xReturn;
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTask );
        rt_base_t level;

        level = rt_hw_interrupt_disable();
        xReturn = ( TaskHookFunction_t ) thread->user_data;
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
        rt_thread_t thread = ( rt_thread_t ) prvGetTCBFromHandle( xTask );

        if( thread->user_data != RT_NULL )
        {
            xReturn = ( ( TaskHookFunction_t ) thread->user_data )( pvParameter );
        }
        else
        {
            xReturn = pdFAIL;
        }

        return xReturn;
    }

#endif /* configUSE_APPLICATION_TASK_TAG */
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
