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
        rt_thread_t thread = ( rt_thread_t ) xTaskToDelete;
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
