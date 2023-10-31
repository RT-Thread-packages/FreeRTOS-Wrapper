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


#ifndef PORTMACRO_H
    #define PORTMACRO_H

    #ifdef __cplusplus
        extern "C" {
    #endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
    #define portCHAR          char
    #define portFLOAT         float
    #define portDOUBLE        double
    #define portLONG          long
    #define portSHORT         short
    #define portSTACK_TYPE    rt_ubase_t
    #define portBASE_TYPE     rt_base_t

    typedef portSTACK_TYPE   StackType_t;
    typedef rt_base_t        BaseType_t;
    typedef rt_ubase_t       UBaseType_t;
    typedef rt_tick_t        TickType_t;
    #define portMAX_DELAY    ( ( TickType_t ) RT_WAITING_FOREVER )

    struct rt_semaphore_wrapper
    {
        struct rt_semaphore sem;
        rt_uint16_t max_value;
    };

/*-----------------------------------------------------------*/

/* Architecture specifics. */
    #define portTICK_PERIOD_MS      ((TickType_t) (1000 / configTICK_RATE_HZ))
    #define portBYTE_ALIGNMENT      RT_ALIGN_SIZE
    #define portPOINTER_SIZE_TYPE   rt_size_t
/*-----------------------------------------------------------*/

/* Scheduler utilities. */
    #define portYIELD()                 rt_thread_yield()
    #define portYIELD_FROM_ISR( x )     rt_thread_yield()

/*-----------------------------------------------------------*/

/* Critical section management. */
    extern void vPortEnterCritical( void );
    extern void vPortExitCritical( void );
    #define portSET_INTERRUPT_MASK_FROM_ISR()         rt_hw_interrupt_disable()
    #define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    rt_hw_interrupt_enable( x )
    #define portDISABLE_INTERRUPTS()                  vPortEnterCritical()
    #define portENABLE_INTERRUPTS()                   vPortExitCritical()
    #define portENTER_CRITICAL()                      vPortEnterCritical()
    #define portEXIT_CRITICAL()                       vPortExitCritical()

/*-----------------------------------------------------------*/

    #define FREERTOS_PRIORITY_TO_RTTHREAD(priority)    ( configMAX_PRIORITIES - 1 - ( priority ) )
    #define RTTHREAD_PRIORITY_TO_FREERTOS(priority)    ( RT_THREAD_PRIORITY_MAX - 1 - ( priority ) )
/* Use this macro to calculate the buffer size when allocating a queue statically
 * To ensure the buffer can fit the desired number of messages
 */
    #define QUEUE_BUFFER_SIZE( uxQueueLength, uxItemSize )  ( ( RT_ALIGN( uxItemSize, RT_ALIGN_SIZE ) + sizeof( void * ) ) * uxQueueLength )

    BaseType_t rt_err_to_freertos(rt_err_t rt_err);

    #ifdef __cplusplus
        }
    #endif

#endif /* PORTMACRO_H */
