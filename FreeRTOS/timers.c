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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* This entire source file will be skipped if the application is not configured
 * to include software timer functionality.  This #if is closed at the very bottom
 * of this file.  If you want to include software timer functionality then ensure
 * configUSE_TIMERS is set to 1 in FreeRTOSConfig.h. */
#if ( configUSE_TIMERS == 1 )

    typedef void (* rt_timer_callback_t)(void *);

/* The definition of the timers themselves. */
    typedef struct tmrTimerControl
    {
        struct rt_timer timer;
        void * pvTimerID;                           /*<< An ID to identify the timer.  This allows the timer to be identified when the same callback is used for multiple timers. */
    } xTIMER;

    typedef xTIMER Timer_t;

/*-----------------------------------------------------------*/

    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

        TimerHandle_t xTimerCreate( const char * const pcTimerName,
                                    const TickType_t xTimerPeriodInTicks,
                                    const UBaseType_t uxAutoReload,
                                    void * const pvTimerID,
                                    TimerCallbackFunction_t pxCallbackFunction )
        {
            Timer_t * pxNewTimer;
            rt_uint8_t flag = RT_TIMER_FLAG_SOFT_TIMER;

            pxNewTimer = ( Timer_t * ) RT_KERNEL_MALLOC( sizeof( Timer_t ) );

            if( pxNewTimer != RT_NULL )
            {
                if ( uxAutoReload != pdFALSE )
                {
                    flag |= RT_TIMER_FLAG_PERIODIC;
                }
                else
                {
                    flag |= RT_TIMER_FLAG_ONE_SHOT;
                }
                rt_timer_init( ( rt_timer_t ) pxNewTimer, pcTimerName, ( rt_timer_callback_t ) pxCallbackFunction, pxNewTimer, xTimerPeriodInTicks, flag );
                pxNewTimer->pvTimerID = pvTimerID;
                /* Mark as dynamic so we can distinguish when deleting */
                ( ( rt_timer_t ) pxNewTimer )->parent.type &= ~RT_Object_Class_Static;
            }

            return pxNewTimer;
        }

    #endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )

        TimerHandle_t xTimerCreateStatic( const char * const pcTimerName,
                                          const TickType_t xTimerPeriodInTicks,
                                          const UBaseType_t uxAutoReload,
                                          void * const pvTimerID,
                                          TimerCallbackFunction_t pxCallbackFunction,
                                          StaticTimer_t * pxTimerBuffer )
        {
            Timer_t * pxNewTimer;
            rt_uint8_t flag = RT_TIMER_FLAG_SOFT_TIMER;

            #if ( configASSERT_DEFINED == 1 )
                {
                    /* Sanity check that the size of the structure used to declare a
                     * variable of type StaticTimer_t equals the size of the real timer
                     * structure. */
                    volatile size_t xSize = sizeof( StaticTimer_t );
                    configASSERT( xSize == sizeof( Timer_t ) );
                    ( void ) xSize; /* Keeps lint quiet when configASSERT() is not defined. */
                }
            #endif /* configASSERT_DEFINED */

            /* A pointer to a StaticTimer_t structure MUST be provided, use it. */
            configASSERT( pxTimerBuffer );
            pxNewTimer = ( Timer_t * ) pxTimerBuffer;

            if( pxNewTimer != NULL )
            {
                if ( uxAutoReload != pdFALSE )
                {
                    flag |= RT_TIMER_FLAG_PERIODIC;
                }
                else
                {
                    flag |= RT_TIMER_FLAG_ONE_SHOT;
                }
                rt_timer_init( ( rt_timer_t ) pxNewTimer, pcTimerName, ( rt_timer_callback_t ) pxCallbackFunction, pxNewTimer, xTimerPeriodInTicks, flag );
                pxNewTimer->pvTimerID = pvTimerID;
            }

            return pxNewTimer;
        }

    #endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

    BaseType_t xTimerGenericCommand( TimerHandle_t xTimer,
                                     const BaseType_t xCommandID,
                                     const TickType_t xOptionalValue,
                                     BaseType_t * const pxHigherPriorityTaskWoken,
                                     const TickType_t xTicksToWait )
    {
        rt_err_t err = -RT_ERROR;

        configASSERT( xTimer );

        if ( ( xCommandID == tmrCOMMAND_START ) || ( xCommandID == tmrCOMMAND_START_FROM_ISR )
             || ( xCommandID == tmrCOMMAND_RESET ) || ( xCommandID == tmrCOMMAND_RESET_FROM_ISR ) )
        {
            err = rt_timer_start( ( rt_timer_t ) xTimer );
        }
        else if ( ( xCommandID == tmrCOMMAND_STOP ) || ( xCommandID == tmrCOMMAND_STOP_FROM_ISR ) )
        {
            err = rt_timer_stop( ( rt_timer_t ) xTimer );
        }
        else if ( ( xCommandID == tmrCOMMAND_CHANGE_PERIOD ) || ( xCommandID == tmrCOMMAND_CHANGE_PERIOD_FROM_ISR ) )
        {
            if ( rt_timer_stop( ( rt_timer_t ) xTimer ) == RT_EOK )
            {
                if ( rt_timer_control( ( rt_timer_t ) xTimer, RT_TIMER_CTRL_SET_TIME, ( void * ) &xOptionalValue ) == RT_EOK )
                {
                    err = rt_timer_start( ( rt_timer_t ) xTimer );
                }
            }
        }
        else if ( xCommandID == tmrCOMMAND_DELETE )
        {
        #if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
            if ( rt_object_is_systemobject( ( rt_object_t ) xTimer ) )
        #endif
            {
            #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
                err = rt_timer_detach( ( rt_timer_t ) xTimer );
            #endif
        #if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
            }
            else
            {
        #endif
            #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
                ( ( rt_timer_t ) xTimer )->parent.type |= RT_Object_Class_Static;
                err = rt_timer_detach( ( rt_timer_t ) xTimer );
                RT_KERNEL_FREE( xTimer );
            #endif
            }
        }

        if ( ( xCommandID >= tmrFIRST_FROM_ISR_COMMAND ) && ( xCommandID <= tmrCOMMAND_CHANGE_PERIOD_FROM_ISR ) && ( pxHigherPriorityTaskWoken != NULL ) )
        {
            *pxHigherPriorityTaskWoken = pdFALSE;
        }

        return rt_err_to_freertos( err );
    }
/*-----------------------------------------------------------*/

    TaskHandle_t xTimerGetTimerDaemonTaskHandle( void )
    {
        return ( TaskHandle_t ) rt_thread_find( "timer" );
    }
/*-----------------------------------------------------------*/

    TickType_t xTimerGetPeriod( TimerHandle_t xTimer )
    {
        Timer_t * pxTimer = xTimer;
        rt_tick_t arg;

        configASSERT( xTimer );
        rt_timer_control( ( rt_timer_t ) pxTimer, RT_TIMER_CTRL_GET_TIME, &arg );

        return ( TickType_t ) arg;
    }
/*-----------------------------------------------------------*/

    void vTimerSetReloadMode( TimerHandle_t xTimer,
                              const UBaseType_t uxAutoReload )
    {
        Timer_t * pxTimer = xTimer;

        configASSERT( xTimer );
        if ( uxAutoReload != pdFALSE )
        {
            rt_timer_control( ( rt_timer_t ) pxTimer, RT_TIMER_CTRL_SET_PERIODIC, RT_NULL );
        }
        else
        {
            rt_timer_control( ( rt_timer_t ) pxTimer, RT_TIMER_CTRL_SET_ONESHOT, RT_NULL );
        }
    }
/*-----------------------------------------------------------*/

    UBaseType_t uxTimerGetReloadMode( TimerHandle_t xTimer )
    {
        Timer_t * pxTimer = xTimer;
        UBaseType_t uxReturn;
        rt_base_t level;

        configASSERT( xTimer );
        level = rt_hw_interrupt_disable();
        if ( ( ( rt_timer_t ) pxTimer )->parent.flag & RT_TIMER_FLAG_PERIODIC )
        {
            uxReturn = ( UBaseType_t ) pdTRUE;
        }
        else
        {
            uxReturn = ( UBaseType_t ) pdFALSE;
        }
        rt_hw_interrupt_enable( level );

        return uxReturn;
    }
/*-----------------------------------------------------------*/

    TickType_t xTimerGetExpiryTime( TimerHandle_t xTimer )
    {
        Timer_t * pxTimer = xTimer;
        TickType_t xReturn;

        configASSERT( xTimer );
        rt_timer_control( ( rt_timer_t ) pxTimer, RT_TIMER_CTRL_GET_REMAIN_TIME, &xReturn );

        return xReturn;
    }
/*-----------------------------------------------------------*/

    const char * pcTimerGetName( TimerHandle_t xTimer )
    {
        Timer_t * pxTimer = xTimer;

        configASSERT( xTimer );
        return ( ( rt_timer_t ) pxTimer )->parent.name;
    }
/*-----------------------------------------------------------*/

    BaseType_t xTimerIsTimerActive( TimerHandle_t xTimer )
    {
        BaseType_t xReturn;
        Timer_t * pxTimer = xTimer;
        rt_uint32_t arg;

        configASSERT( xTimer );

        rt_timer_control( ( rt_timer_t ) pxTimer, RT_TIMER_CTRL_GET_STATE, &arg );
        if ( arg == RT_TIMER_FLAG_ACTIVATED )
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

    void * pvTimerGetTimerID( const TimerHandle_t xTimer )
    {
        Timer_t * const pxTimer = xTimer;
        void * pvReturn;
        rt_base_t level;

        configASSERT( xTimer );

        level = rt_hw_interrupt_disable();
        pvReturn = pxTimer->pvTimerID;
        rt_hw_interrupt_enable( level );

        return pvReturn;
    }
/*-----------------------------------------------------------*/

    void vTimerSetTimerID( TimerHandle_t xTimer,
                           void * pvNewID )
    {
        Timer_t * const pxTimer = xTimer;
        rt_base_t level;

        configASSERT( xTimer );

        level = rt_hw_interrupt_disable();
        pxTimer->pvTimerID = pvNewID;
        rt_hw_interrupt_enable( level );
    }
/*-----------------------------------------------------------*/

#endif /* configUSE_TIMERS == 1 */
