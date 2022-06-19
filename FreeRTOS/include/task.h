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


#ifndef INC_TASK_H
#define INC_TASK_H

#ifndef INC_FREERTOS_H
    #error "include FreeRTOS.h must appear in source files before include task.h"
#endif

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/*-----------------------------------------------------------
* MACROS AND DEFINITIONS
*----------------------------------------------------------*/

/*
 * If tskKERNEL_VERSION_NUMBER ends with + it represents the version in development
 * after the numbered release.
 *
 * The tskKERNEL_VERSION_MAJOR, tskKERNEL_VERSION_MINOR, tskKERNEL_VERSION_BUILD
 * values will reflect the last released version number.
 */
#define tskKERNEL_VERSION_NUMBER       "V10.4.6"
#define tskKERNEL_VERSION_MAJOR        10
#define tskKERNEL_VERSION_MINOR        4
#define tskKERNEL_VERSION_BUILD        6

/**
 * task. h
 *
 * Type by which tasks are referenced.  For example, a call to xTaskCreate
 * returns (via a pointer parameter) an TaskHandle_t variable that can then
 * be used as a parameter to vTaskDelete to delete the task.
 *
 * \defgroup TaskHandle_t TaskHandle_t
 * \ingroup Tasks
 */
struct tskTaskControlBlock; /* The old naming convention is used to prevent breaking kernel aware debuggers. */
typedef struct tskTaskControlBlock * TaskHandle_t;

/**
 * task. h
 *
 * Macro for forcing a context switch.
 *
 * \defgroup taskYIELD taskYIELD
 * \ingroup SchedulerControl
 */
#define taskYIELD()                        portYIELD()

/**
 * task. h
 *
 * Macro to mark the start of a critical code region.  Preemptive context
 * switches cannot occur when in a critical region.
 *
 * NOTE: This may alter the stack (depending on the portable implementation)
 * so must be used with care!
 *
 * \defgroup taskENTER_CRITICAL taskENTER_CRITICAL
 * \ingroup SchedulerControl
 */
#define taskENTER_CRITICAL()               portENTER_CRITICAL()
#define taskENTER_CRITICAL_FROM_ISR()      portSET_INTERRUPT_MASK_FROM_ISR()

/**
 * task. h
 *
 * Macro to mark the end of a critical code region.  Preemptive context
 * switches cannot occur when in a critical region.
 *
 * NOTE: This may alter the stack (depending on the portable implementation)
 * so must be used with care!
 *
 * \defgroup taskEXIT_CRITICAL taskEXIT_CRITICAL
 * \ingroup SchedulerControl
 */
#define taskEXIT_CRITICAL()                portEXIT_CRITICAL()
#define taskEXIT_CRITICAL_FROM_ISR( x )    portCLEAR_INTERRUPT_MASK_FROM_ISR( x )

/**
 * task. h
 *
 * Macro to disable all maskable interrupts.
 *
 * \defgroup taskDISABLE_INTERRUPTS taskDISABLE_INTERRUPTS
 * \ingroup SchedulerControl
 */
#define taskDISABLE_INTERRUPTS()           portDISABLE_INTERRUPTS()

/**
 * task. h
 *
 * Macro to enable microcontroller interrupts.
 *
 * \defgroup taskENABLE_INTERRUPTS taskENABLE_INTERRUPTS
 * \ingroup SchedulerControl
 */
#define taskENABLE_INTERRUPTS()            portENABLE_INTERRUPTS()

/*-----------------------------------------------------------
* TASK CREATION API
*----------------------------------------------------------*/

/**
 * task. h
 * @code{c}
 * BaseType_t xTaskCreate(
 *                            TaskFunction_t pxTaskCode,
 *                            const char *pcName,
 *                            configSTACK_DEPTH_TYPE usStackDepth,
 *                            void *pvParameters,
 *                            UBaseType_t uxPriority,
 *                            TaskHandle_t *pxCreatedTask
 *                        );
 * @endcode
 *
 * Create a new task and add it to the list of tasks that are ready to run.
 *
 * Internally, within the FreeRTOS implementation, tasks use two blocks of
 * memory.  The first block is used to hold the task's data structures.  The
 * second block is used by the task as its stack.  If a task is created using
 * xTaskCreate() then both blocks of memory are automatically dynamically
 * allocated inside the xTaskCreate() function.  (see
 * https://www.FreeRTOS.org/a00111.html).  If a task is created using
 * xTaskCreateStatic() then the application writer must provide the required
 * memory.  xTaskCreateStatic() therefore allows a task to be created without
 * using any dynamic memory allocation.
 *
 * See xTaskCreateStatic() for a version that does not use any dynamic memory
 * allocation.
 *
 * xTaskCreate() can only be used to create a task that has unrestricted
 * access to the entire microcontroller memory map.  Systems that include MPU
 * support can alternatively create an MPU constrained task using
 * xTaskCreateRestricted().
 *
 * @param pxTaskCode Pointer to the task entry function.  Tasks
 * must be implemented to never return (i.e. continuous loop).
 *
 * @param pcName A descriptive name for the task.  This is mainly used to
 * facilitate debugging.  Max length defined by configMAX_TASK_NAME_LEN - default
 * is 16.
 *
 * @param usStackDepth The size of the task stack specified as the number of
 * variables the stack can hold - not the number of bytes.  For example, if
 * the stack is 16 bits wide and usStackDepth is defined as 100, 200 bytes
 * will be allocated for stack storage.
 *
 * @param pvParameters Pointer that will be used as the parameter for the task
 * being created.
 *
 * @param uxPriority The priority at which the task should run.  Systems that
 * include MPU support can optionally create tasks in a privileged (system)
 * mode by setting bit portPRIVILEGE_BIT of the priority parameter.  For
 * example, to create a privileged task at priority 2 the uxPriority parameter
 * should be set to ( 2 | portPRIVILEGE_BIT ).
 *
 * @param pxCreatedTask Used to pass back a handle by which the created task
 * can be referenced.
 *
 * @return pdPASS if the task was successfully created and added to a ready
 * list, otherwise an error code defined in the file projdefs.h
 *
 * Example usage:
 * @code{c}
 * // Task to be created.
 * void vTaskCode( void * pvParameters )
 * {
 *   for( ;; )
 *   {
 *       // Task code goes here.
 *   }
 * }
 *
 * // Function that creates a task.
 * void vOtherFunction( void )
 * {
 * static uint8_t ucParameterToPass;
 * TaskHandle_t xHandle = NULL;
 *
 *   // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
 *   // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
 *   // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
 *   // the new task attempts to access it.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
 *   configASSERT( xHandle );
 *
 *   // Use the handle to delete the task.
 *   if( xHandle != NULL )
 *   {
 *      vTaskDelete( xHandle );
 *   }
 * }
 * @endcode
 * \defgroup xTaskCreate xTaskCreate
 * \ingroup Tasks
 */
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
    BaseType_t xTaskCreate( TaskFunction_t pxTaskCode,
                            const char * const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                            const configSTACK_DEPTH_TYPE usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pxCreatedTask );
#endif

/**
 * task. h
 * @code{c}
 * TaskHandle_t xTaskCreateStatic( TaskFunction_t pxTaskCode,
 *                               const char *pcName,
 *                               uint32_t ulStackDepth,
 *                               void *pvParameters,
 *                               UBaseType_t uxPriority,
 *                               StackType_t *puxStackBuffer,
 *                               StaticTask_t *pxTaskBuffer );
 * @endcode
 *
 * Create a new task and add it to the list of tasks that are ready to run.
 *
 * Internally, within the FreeRTOS implementation, tasks use two blocks of
 * memory.  The first block is used to hold the task's data structures.  The
 * second block is used by the task as its stack.  If a task is created using
 * xTaskCreate() then both blocks of memory are automatically dynamically
 * allocated inside the xTaskCreate() function.  (see
 * https://www.FreeRTOS.org/a00111.html).  If a task is created using
 * xTaskCreateStatic() then the application writer must provide the required
 * memory.  xTaskCreateStatic() therefore allows a task to be created without
 * using any dynamic memory allocation.
 *
 * @param pxTaskCode Pointer to the task entry function.  Tasks
 * must be implemented to never return (i.e. continuous loop).
 *
 * @param pcName A descriptive name for the task.  This is mainly used to
 * facilitate debugging.  The maximum length of the string is defined by
 * configMAX_TASK_NAME_LEN in FreeRTOSConfig.h.
 *
 * @param ulStackDepth The size of the task stack specified as the number of
 * variables the stack can hold - not the number of bytes.  For example, if
 * the stack is 32-bits wide and ulStackDepth is defined as 100 then 400 bytes
 * will be allocated for stack storage.
 *
 * @param pvParameters Pointer that will be used as the parameter for the task
 * being created.
 *
 * @param uxPriority The priority at which the task will run.
 *
 * @param puxStackBuffer Must point to a StackType_t array that has at least
 * ulStackDepth indexes - the array will then be used as the task's stack,
 * removing the need for the stack to be allocated dynamically.
 *
 * @param pxTaskBuffer Must point to a variable of type StaticTask_t, which will
 * then be used to hold the task's data structures, removing the need for the
 * memory to be allocated dynamically.
 *
 * @return If neither puxStackBuffer nor pxTaskBuffer are NULL, then the task
 * will be created and a handle to the created task is returned.  If either
 * puxStackBuffer or pxTaskBuffer are NULL then the task will not be created and
 * NULL is returned.
 *
 * Example usage:
 * @code{c}
 *
 *  // Dimensions of the buffer that the task being created will use as its stack.
 *  // NOTE:  This is the number of words the stack will hold, not the number of
 *  // bytes.  For example, if each stack item is 32-bits, and this is set to 100,
 *  // then 400 bytes (100 * 32-bits) will be allocated.
 #define STACK_SIZE 200
 *
 *  // Structure that will hold the TCB of the task being created.
 *  StaticTask_t xTaskBuffer;
 *
 *  // Buffer that the task being created will use as its stack.  Note this is
 *  // an array of StackType_t variables.  The size of StackType_t is dependent on
 *  // the RTOS port.
 *  StackType_t xStack[ STACK_SIZE ];
 *
 *  // Function that implements the task being created.
 *  void vTaskCode( void * pvParameters )
 *  {
 *      // The parameter value is expected to be 1 as 1 is passed in the
 *      // pvParameters value in the call to xTaskCreateStatic().
 *      configASSERT( ( uint32_t ) pvParameters == 1UL );
 *
 *      for( ;; )
 *      {
 *          // Task code goes here.
 *      }
 *  }
 *
 *  // Function that creates a task.
 *  void vOtherFunction( void )
 *  {
 *      TaskHandle_t xHandle = NULL;
 *
 *      // Create the task without using any dynamic memory allocation.
 *      xHandle = xTaskCreateStatic(
 *                    vTaskCode,       // Function that implements the task.
 *                    "NAME",          // Text name for the task.
 *                    STACK_SIZE,      // Stack size in words, not bytes.
 *                    ( void * ) 1,    // Parameter passed into the task.
 *                    tskIDLE_PRIORITY,// Priority at which the task is created.
 *                    xStack,          // Array to use as the task's stack.
 *                    &xTaskBuffer );  // Variable to hold the task's data structure.
 *
 *      // puxStackBuffer and pxTaskBuffer were not NULL, so the task will have
 *      // been created, and xHandle will be the task's handle.  Use the handle
 *      // to suspend the task.
 *      vTaskSuspend( xHandle );
 *  }
 * @endcode
 * \defgroup xTaskCreateStatic xTaskCreateStatic
 * \ingroup Tasks
 */
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
    TaskHandle_t xTaskCreateStatic( TaskFunction_t pxTaskCode,
                                    const char * const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                                    const uint32_t ulStackDepth,
                                    void * const pvParameters,
                                    UBaseType_t uxPriority,
                                    StackType_t * const puxStackBuffer,
                                    StaticTask_t * const pxTaskBuffer );
#endif /* configSUPPORT_STATIC_ALLOCATION */

/**
 * task. h
 * @code{c}
 * void vTaskDelete( TaskHandle_t xTaskToDelete );
 * @endcode
 *
 * INCLUDE_vTaskDelete must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * Remove a task from the RTOS real time kernel's management.  The task being
 * deleted will be removed from all ready, blocked, suspended and event lists.
 *
 * NOTE:  The idle task is responsible for freeing the kernel allocated
 * memory from tasks that have been deleted.  It is therefore important that
 * the idle task is not starved of microcontroller processing time if your
 * application makes any calls to vTaskDelete ().  Memory allocated by the
 * task code is not automatically freed, and should be freed before the task
 * is deleted.
 *
 * See the demo application file death.c for sample code that utilises
 * vTaskDelete ().
 *
 * @param xTaskToDelete The handle of the task to be deleted.  Passing NULL will
 * cause the calling task to be deleted.
 *
 * Example usage:
 * @code{c}
 * void vOtherFunction( void )
 * {
 * TaskHandle_t xHandle;
 *
 *   // Create the task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // Use the handle to delete the task.
 *   vTaskDelete( xHandle );
 * }
 * @endcode
 * \defgroup vTaskDelete vTaskDelete
 * \ingroup Tasks
 */
void vTaskDelete( TaskHandle_t xTaskToDelete );

/*-----------------------------------------------------------
* TASK CONTROL API
*----------------------------------------------------------*/

/**
 * task. h
 * @code{c}
 * void vTaskDelay( const TickType_t xTicksToDelay );
 * @endcode
 *
 * Delay a task for a given number of ticks.  The actual time that the
 * task remains blocked depends on the tick rate.  The constant
 * portTICK_PERIOD_MS can be used to calculate real time from the tick
 * rate - with the resolution of one tick period.
 *
 * INCLUDE_vTaskDelay must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 *
 * vTaskDelay() specifies a time at which the task wishes to unblock relative to
 * the time at which vTaskDelay() is called.  For example, specifying a block
 * period of 100 ticks will cause the task to unblock 100 ticks after
 * vTaskDelay() is called.  vTaskDelay() does not therefore provide a good method
 * of controlling the frequency of a periodic task as the path taken through the
 * code, as well as other task and interrupt activity, will affect the frequency
 * at which vTaskDelay() gets called and therefore the time at which the task
 * next executes.  See xTaskDelayUntil() for an alternative API function designed
 * to facilitate fixed frequency execution.  It does this by specifying an
 * absolute time (rather than a relative time) at which the calling task should
 * unblock.
 *
 * @param xTicksToDelay The amount of time, in tick periods, that
 * the calling task should block.
 *
 * Example usage:
 *
 * void vTaskFunction( void * pvParameters )
 * {
 * // Block for 500ms.
 * const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
 *
 *   for( ;; )
 *   {
 *       // Simply toggle the LED every 500ms, blocking between each toggle.
 *       vToggleLED();
 *       vTaskDelay( xDelay );
 *   }
 * }
 *
 * \defgroup vTaskDelay vTaskDelay
 * \ingroup TaskCtrl
 */
void vTaskDelay( const TickType_t xTicksToDelay );

/**
 * task. h
 * @code{c}
 * BaseType_t xTaskDelayUntil( TickType_t *pxPreviousWakeTime, const TickType_t xTimeIncrement );
 * @endcode
 *
 * INCLUDE_xTaskDelayUntil must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * Delay a task until a specified time.  This function can be used by periodic
 * tasks to ensure a constant execution frequency.
 *
 * This function differs from vTaskDelay () in one important aspect:  vTaskDelay () will
 * cause a task to block for the specified number of ticks from the time vTaskDelay () is
 * called.  It is therefore difficult to use vTaskDelay () by itself to generate a fixed
 * execution frequency as the time between a task starting to execute and that task
 * calling vTaskDelay () may not be fixed [the task may take a different path though the
 * code between calls, or may get interrupted or preempted a different number of times
 * each time it executes].
 *
 * Whereas vTaskDelay () specifies a wake time relative to the time at which the function
 * is called, xTaskDelayUntil () specifies the absolute (exact) time at which it wishes to
 * unblock.
 *
 * The macro pdMS_TO_TICKS() can be used to calculate the number of ticks from a
 * time specified in milliseconds with a resolution of one tick period.
 *
 * @param pxPreviousWakeTime Pointer to a variable that holds the time at which the
 * task was last unblocked.  The variable must be initialised with the current time
 * prior to its first use (see the example below).  Following this the variable is
 * automatically updated within xTaskDelayUntil ().
 *
 * @param xTimeIncrement The cycle time period.  The task will be unblocked at
 * time *pxPreviousWakeTime + xTimeIncrement.  Calling xTaskDelayUntil with the
 * same xTimeIncrement parameter value will cause the task to execute with
 * a fixed interface period.
 *
 * @return Value which can be used to check whether the task was actually delayed.
 * Will be pdTRUE if the task way delayed and pdFALSE otherwise.  A task will not
 * be delayed if the next expected wake time is in the past.
 *
 * Example usage:
 * @code{c}
 * // Perform an action every 10 ticks.
 * void vTaskFunction( void * pvParameters )
 * {
 * TickType_t xLastWakeTime;
 * const TickType_t xFrequency = 10;
 * BaseType_t xWasDelayed;
 *
 *     // Initialise the xLastWakeTime variable with the current time.
 *     xLastWakeTime = xTaskGetTickCount ();
 *     for( ;; )
 *     {
 *         // Wait for the next cycle.
 *         xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
 *
 *         // Perform action here. xWasDelayed value can be used to determine
 *         // whether a deadline was missed if the code here took too long.
 *     }
 * }
 * @endcode
 * \defgroup xTaskDelayUntil xTaskDelayUntil
 * \ingroup TaskCtrl
 */
BaseType_t xTaskDelayUntil( TickType_t * const pxPreviousWakeTime,
                            const TickType_t xTimeIncrement );

/*
 * vTaskDelayUntil() is the older version of xTaskDelayUntil() and does not
 * return a value.
 */
#define vTaskDelayUntil( pxPreviousWakeTime, xTimeIncrement )           \
    {                                                                   \
        ( void ) xTaskDelayUntil( pxPreviousWakeTime, xTimeIncrement ); \
    }

/*-----------------------------------------------------------
* SCHEDULER CONTROL
*----------------------------------------------------------*/

/**
 * task. h
 * @code{c}
 * void vTaskSuspendAll( void );
 * @endcode
 *
 * Suspends the scheduler without disabling interrupts.  Context switches will
 * not occur while the scheduler is suspended.
 *
 * After calling vTaskSuspendAll () the calling task will continue to execute
 * without risk of being swapped out until a call to xTaskResumeAll () has been
 * made.
 *
 * API functions that have the potential to cause a context switch (for example,
 * xTaskDelayUntil(), xQueueSend(), etc.) must not be called while the scheduler
 * is suspended.
 *
 * Example usage:
 * @code{c}
 * void vTask1( void * pvParameters )
 * {
 *   for( ;; )
 *   {
 *       // Task code goes here.
 *
 *       // ...
 *
 *       // At some point the task wants to perform a long operation during
 *       // which it does not want to get swapped out.  It cannot use
 *       // taskENTER_CRITICAL ()/taskEXIT_CRITICAL () as the length of the
 *       // operation may cause interrupts to be missed - including the
 *       // ticks.
 *
 *       // Prevent the real time kernel swapping out the task.
 *       vTaskSuspendAll ();
 *
 *       // Perform the operation here.  There is no need to use critical
 *       // sections as we have all the microcontroller processing time.
 *       // During this time interrupts will still operate and the kernel
 *       // tick count will be maintained.
 *
 *       // ...
 *
 *       // The operation is complete.  Restart the kernel.
 *       xTaskResumeAll ();
 *   }
 * }
 * @endcode
 * \defgroup vTaskSuspendAll vTaskSuspendAll
 * \ingroup SchedulerControl
 */
void vTaskSuspendAll( void );

/**
 * task. h
 * @code{c}
 * BaseType_t xTaskResumeAll( void );
 * @endcode
 *
 * Resumes scheduler activity after it was suspended by a call to
 * vTaskSuspendAll().
 *
 * xTaskResumeAll() only resumes the scheduler.  It does not unsuspend tasks
 * that were previously suspended by a call to vTaskSuspend().
 *
 * @return If resuming the scheduler caused a context switch then pdTRUE is
 *         returned, otherwise pdFALSE is returned.
 *
 * Example usage:
 * @code{c}
 * void vTask1( void * pvParameters )
 * {
 *   for( ;; )
 *   {
 *       // Task code goes here.
 *
 *       // ...
 *
 *       // At some point the task wants to perform a long operation during
 *       // which it does not want to get swapped out.  It cannot use
 *       // taskENTER_CRITICAL ()/taskEXIT_CRITICAL () as the length of the
 *       // operation may cause interrupts to be missed - including the
 *       // ticks.
 *
 *       // Prevent the real time kernel swapping out the task.
 *       vTaskSuspendAll ();
 *
 *       // Perform the operation here.  There is no need to use critical
 *       // sections as we have all the microcontroller processing time.
 *       // During this time interrupts will still operate and the real
 *       // time kernel tick count will be maintained.
 *
 *       // ...
 *
 *       // The operation is complete.  Restart the kernel.  We want to force
 *       // a context switch - but there is no point if resuming the scheduler
 *       // caused a context switch already.
 *       if( !xTaskResumeAll () )
 *       {
 *            taskYIELD ();
 *       }
 *   }
 * }
 * @endcode
 * \defgroup xTaskResumeAll xTaskResumeAll
 * \ingroup SchedulerControl
 */
BaseType_t xTaskResumeAll( void );

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */
#endif /* INC_TASK_H */
