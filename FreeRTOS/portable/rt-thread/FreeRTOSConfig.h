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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef PKG_FREERTOS_USING_PRELOAD_CONFIG
#include <FreeRTOSPreloadConfig.h>
#endif

/* The following options are read-only */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_16_BIT_TICKS                  0
#define configUSE_FREERTOS_PROVIDED_HEAP        1

/* The following read-only options are controlled by rtconfig.h. */
#define configTICK_RATE_HZ                      RT_TICK_PER_SECOND
#define configMAX_PRIORITIES                    RT_THREAD_PRIORITY_MAX
#define configMAX_TASK_NAME_LEN                 RT_NAME_MAX

#ifdef RT_USING_TIMER_SOFT
    #define configUSE_TIMERS                    1
    #define configTIMER_TASK_PRIORITY           (RT_THREAD_PRIORITY_MAX - 1 - RT_TIMER_THREAD_PRIO)
    /* RT-Thread does not use a timer queue. This option is not used. */
    #define configTIMER_QUEUE_LENGTH            0
    #define configTIMER_TASK_STACK_DEPTH        RT_TIMER_THREAD_STACK_SIZE
#endif

/* These options can be modified to selectively disable recursive mutex */
/* Take effect only if RT_USING_MUTEX is defined */
#ifdef RT_USING_MUTEX
    #define configUSE_RECURSIVE_MUTEXES         1
    #define configUSE_MUTEXES                   1
#endif

/* These options can be modified to selectively disable counting semaphore */
/* Take effect only if RT_USING_SEMAPHORE is defined */
#ifdef RT_USING_SEMAPHORE
    #define configUSE_COUNTING_SEMAPHORES       1
#endif

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         1
#ifdef RT_USING_HEAP
    #define configSUPPORT_DYNAMIC_ALLOCATION    1
    #define configTOTAL_HEAP_SIZE               10240
    #define configAPPLICATION_ALLOCATED_HEAP    0
#endif

/* Hook functions are not supported. */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* The following features are not supported. */
#define INCLUDE_xTimerPendFunctionCall          0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
#define configUSE_CO_ROUTINES                   0
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_QUEUE_SETS                    0
#define configUSE_NEWLIB_REENTRANT              0
#define configUSE_TICKLESS_IDLE                 0
#define configQUEUE_REGISTRY_SIZE               0
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Can be modified */
#define configMINIMAL_STACK_SIZE                128

/* Optional functions */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xSemaphoreGetMutexHolder        1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_uxTaskGetStackHighWaterMark2    1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define configUSE_APPLICATION_TASK_TAG          1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3

/* Other definitions can go here, e.g. configAssert*/

#endif /* FREERTOS_CONFIG_H */
