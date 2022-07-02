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

/*
 * Defines the prototype to which the application task hook function must
 * conform.
 */
typedef BaseType_t (* TaskHookFunction_t)( void * );

/* Task states returned by eTaskGetState. */
typedef enum
{
    eRunning = 0, /* A task is querying the state of itself, so must be running. */
    eReady,       /* The task being queried is in a ready or pending ready list. */
    eBlocked,     /* The task being queried is in the Blocked state. */
    eSuspended,   /* The task being queried is in the Suspended state, or is in the Blocked state with an infinite time out. */
    eDeleted,     /* The task being queried has been deleted, but its TCB has not yet been freed. */
    eInvalid      /* Used as an 'invalid state' value. */
} eTaskState;

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

/**
 * task. h
 * @code{c}
 * BaseType_t xTaskAbortDelay( TaskHandle_t xTask );
 * @endcode
 *
 * INCLUDE_xTaskAbortDelay must be defined as 1 in FreeRTOSConfig.h for this
 * function to be available.
 *
 * A task will enter the Blocked state when it is waiting for an event.  The
 * event it is waiting for can be a temporal event (waiting for a time), such
 * as when vTaskDelay() is called, or an event on an object, such as when
 * xQueueReceive() or ulTaskNotifyTake() is called.  If the handle of a task
 * that is in the Blocked state is used in a call to xTaskAbortDelay() then the
 * task will leave the Blocked state, and return from whichever function call
 * placed the task into the Blocked state.
 *
 * There is no 'FromISR' version of this function as an interrupt would need to
 * know which object a task was blocked on in order to know which actions to
 * take.  For example, if the task was blocked on a queue the interrupt handler
 * would then need to know if the queue was locked.
 *
 * @param xTask The handle of the task to remove from the Blocked state.
 *
 * @return If the task referenced by xTask was not in the Blocked state then
 * pdFAIL is returned.  Otherwise pdPASS is returned.
 *
 * \defgroup xTaskAbortDelay xTaskAbortDelay
 * \ingroup TaskCtrl
 */
BaseType_t xTaskAbortDelay( TaskHandle_t xTask );

/**
 * task. h
 * @code{c}
 * UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask );
 * @endcode
 *
 * INCLUDE_uxTaskPriorityGet must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * Obtain the priority of any task.
 *
 * @param xTask Handle of the task to be queried.  Passing a NULL
 * handle results in the priority of the calling task being returned.
 *
 * @return The priority of xTask.
 *
 * Example usage:
 * @code{c}
 * void vAFunction( void )
 * {
 * TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to obtain the priority of the created task.
 *   // It was created with tskIDLE_PRIORITY, but may have changed
 *   // it itself.
 *   if( uxTaskPriorityGet( xHandle ) != tskIDLE_PRIORITY )
 *   {
 *       // The task has changed it's priority.
 *   }
 *
 *   // ...
 *
 *   // Is our priority higher than the created task?
 *   if( uxTaskPriorityGet( xHandle ) < uxTaskPriorityGet( NULL ) )
 *   {
 *       // Our priority (obtained using NULL handle) is higher.
 *   }
 * }
 * @endcode
 * \defgroup uxTaskPriorityGet uxTaskPriorityGet
 * \ingroup TaskCtrl
 */
UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask );

/**
 * task. h
 * @code{c}
 * UBaseType_t uxTaskPriorityGetFromISR( const TaskHandle_t xTask );
 * @endcode
 *
 * A version of uxTaskPriorityGet() that can be used from an ISR.
 */
UBaseType_t uxTaskPriorityGetFromISR( const TaskHandle_t xTask );

/**
 * task. h
 * @code{c}
 * eTaskState eTaskGetState( TaskHandle_t xTask );
 * @endcode
 *
 * INCLUDE_eTaskGetState must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * Obtain the state of any task.  States are encoded by the eTaskState
 * enumerated type.
 *
 * @param xTask Handle of the task to be queried.
 *
 * @return The state of xTask at the time the function was called.  Note the
 * state of the task might change between the function being called, and the
 * functions return value being tested by the calling task.
 */
eTaskState eTaskGetState( TaskHandle_t xTask );

/**
 * task. h
 * @code{c}
 * void vTaskPrioritySet( TaskHandle_t xTask, UBaseType_t uxNewPriority );
 * @endcode
 *
 * INCLUDE_vTaskPrioritySet must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * Set the priority of any task.
 *
 * A context switch will occur before the function returns if the priority
 * being set is higher than the currently executing task.
 *
 * @param xTask Handle to the task for which the priority is being set.
 * Passing a NULL handle results in the priority of the calling task being set.
 *
 * @param uxNewPriority The priority to which the task will be set.
 *
 * Example usage:
 * @code{c}
 * void vAFunction( void )
 * {
 * TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to raise the priority of the created task.
 *   vTaskPrioritySet( xHandle, tskIDLE_PRIORITY + 1 );
 *
 *   // ...
 *
 *   // Use a NULL handle to raise our priority to the same value.
 *   vTaskPrioritySet( NULL, tskIDLE_PRIORITY + 1 );
 * }
 * @endcode
 * \defgroup vTaskPrioritySet vTaskPrioritySet
 * \ingroup TaskCtrl
 */
void vTaskPrioritySet( TaskHandle_t xTask,
                       UBaseType_t uxNewPriority );

/**
 * task. h
 * @code{c}
 * void vTaskSuspend( TaskHandle_t xTaskToSuspend );
 * @endcode
 *
 * INCLUDE_vTaskSuspend must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * Suspend any task.  When suspended a task will never get any microcontroller
 * processing time, no matter what its priority.
 *
 * Calls to vTaskSuspend are not accumulative -
 * i.e. calling vTaskSuspend () twice on the same task still only requires one
 * call to vTaskResume () to ready the suspended task.
 *
 * RT-Thread only supports suspending the current running thread.
 * This function must be called with NULL as the parameter.
 *
 * @param xTaskToSuspend Handle to the task being suspended.  Passing a NULL
 * handle will cause the calling task to be suspended.
 *
 * Example usage:
 * @code{c}
 * void vAFunction( void )
 * {
 * TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to suspend the created task.
 *   vTaskSuspend( xHandle );
 *
 *   // ...
 *
 *   // The created task will not run during this period, unless
 *   // another task calls vTaskResume( xHandle ).
 *
 *   //...
 *
 *
 *   // Suspend ourselves.
 *   vTaskSuspend( NULL );
 *
 *   // We cannot get here unless another task calls vTaskResume
 *   // with our handle as the parameter.
 * }
 * @endcode
 * \defgroup vTaskSuspend vTaskSuspend
 * \ingroup TaskCtrl
 */
void vTaskSuspend( TaskHandle_t xTaskToSuspend );

/**
 * task. h
 * @code{c}
 * void vTaskResume( TaskHandle_t xTaskToResume );
 * @endcode
 *
 * INCLUDE_vTaskSuspend must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * Resumes a suspended task.
 *
 * A task that has been suspended by one or more calls to vTaskSuspend ()
 * will be made available for running again by a single call to
 * vTaskResume ().
 *
 * @param xTaskToResume Handle to the task being readied.
 *
 * Example usage:
 * @code{c}
 * void vAFunction( void )
 * {
 * TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to suspend the created task.
 *   vTaskSuspend( xHandle );
 *
 *   // ...
 *
 *   // The created task will not run during this period, unless
 *   // another task calls vTaskResume( xHandle ).
 *
 *   //...
 *
 *
 *   // Resume the suspended task ourselves.
 *   vTaskResume( xHandle );
 *
 *   // The created task will once again get microcontroller processing
 *   // time in accordance with its priority within the system.
 * }
 * @endcode
 * \defgroup vTaskResume vTaskResume
 * \ingroup TaskCtrl
 */
void vTaskResume( TaskHandle_t xTaskToResume );

/**
 * task. h
 * @code{c}
 * void xTaskResumeFromISR( TaskHandle_t xTaskToResume );
 * @endcode
 *
 * INCLUDE_xTaskResumeFromISR must be defined as 1 for this function to be
 * available.  See the configuration section for more information.
 *
 * An implementation of vTaskResume() that can be called from within an ISR.
 *
 * A task that has been suspended by one or more calls to vTaskSuspend ()
 * will be made available for running again by a single call to
 * xTaskResumeFromISR ().
 *
 * xTaskResumeFromISR() should not be used to synchronise a task with an
 * interrupt if there is a chance that the interrupt could arrive prior to the
 * task being suspended - as this can lead to interrupts being missed. Use of a
 * semaphore as a synchronisation mechanism would avoid this eventuality.
 *
 * @param xTaskToResume Handle to the task being readied.
 *
 * @return pdTRUE if resuming the task should result in a context switch,
 * otherwise pdFALSE. This is used by the ISR to determine if a context switch
 * may be required following the ISR.
 *
 * \defgroup vTaskResumeFromISR vTaskResumeFromISR
 * \ingroup TaskCtrl
 */
BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume );

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

/*-----------------------------------------------------------
* TASK UTILITIES
*----------------------------------------------------------*/

/**
 * task. h
 * @code{c}
 * TickType_t xTaskGetTickCount( void );
 * @endcode
 *
 * @return The count of ticks since vTaskStartScheduler was called.
 *
 * \defgroup xTaskGetTickCount xTaskGetTickCount
 * \ingroup TaskUtils
 */
TickType_t xTaskGetTickCount( void );

/**
 * task. h
 * @code{c}
 * TickType_t xTaskGetTickCountFromISR( void );
 * @endcode
 *
 * @return The count of ticks since vTaskStartScheduler was called.
 *
 * This is a version of xTaskGetTickCount() that is safe to be called from an
 * ISR - provided that TickType_t is the natural word size of the
 * microcontroller being used or interrupt nesting is either not supported or
 * not being used.
 *
 * \defgroup xTaskGetTickCountFromISR xTaskGetTickCountFromISR
 * \ingroup TaskUtils
 */
TickType_t xTaskGetTickCountFromISR( void );

/**
 * task. h
 * @code{c}
 * uint16_t uxTaskGetNumberOfTasks( void );
 * @endcode
 *
 * @return The number of tasks that the real time kernel is currently managing.
 * This includes all ready, blocked and suspended tasks.  A task that
 * has been deleted but not yet freed by the idle task will also be
 * included in the count.
 *
 * \defgroup uxTaskGetNumberOfTasks uxTaskGetNumberOfTasks
 * \ingroup TaskUtils
 */
UBaseType_t uxTaskGetNumberOfTasks( void );

/**
 * task. h
 * @code{c}
 * char *pcTaskGetName( TaskHandle_t xTaskToQuery );
 * @endcode
 *
 * @return The text (human readable) name of the task referenced by the handle
 * xTaskToQuery.  A task can query its own name by either passing in its own
 * handle, or by setting xTaskToQuery to NULL.
 *
 * \defgroup pcTaskGetName pcTaskGetName
 * \ingroup TaskUtils
 */
char * pcTaskGetName( TaskHandle_t xTaskToQuery ); /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

/**
 * task. h
 * @code{c}
 * TaskHandle_t xTaskGetHandle( const char *pcNameToQuery );
 * @endcode
 *
 * NOTE:  This function takes a relatively long time to complete and should be
 * used sparingly.
 *
 * @return The handle of the task that has the human readable name pcNameToQuery.
 * NULL is returned if no matching name is found.  INCLUDE_xTaskGetHandle
 * must be set to 1 in FreeRTOSConfig.h for pcTaskGetHandle() to be available.
 *
 * \defgroup pcTaskGetHandle pcTaskGetHandle
 * \ingroup TaskUtils
 */
TaskHandle_t xTaskGetHandle( const char * pcNameToQuery ); /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

/**
 * task.h
 * @code{c}
 * UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask );
 * @endcode
 *
 * INCLUDE_uxTaskGetStackHighWaterMark must be set to 1 in FreeRTOSConfig.h for
 * this function to be available.
 *
 * Returns the high water mark of the stack associated with xTask.  That is,
 * the minimum free stack space there has been (in words, so on a 32 bit machine
 * a value of 1 means 4 bytes) since the task started.  The smaller the returned
 * number the closer the task has come to overflowing its stack.
 *
 * uxTaskGetStackHighWaterMark() and uxTaskGetStackHighWaterMark2() are the
 * same except for their return type.  Using configSTACK_DEPTH_TYPE allows the
 * user to determine the return type.  It gets around the problem of the value
 * overflowing on 8-bit types without breaking backward compatibility for
 * applications that expect an 8-bit return type.
 *
 * @param xTask Handle of the task associated with the stack to be checked.
 * Set xTask to NULL to check the stack of the calling task.
 *
 * @return The smallest amount of free stack space there has been (in words, so
 * actual spaces on the stack rather than bytes) since the task referenced by
 * xTask was created.
 */
UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask );

/**
 * task.h
 * @code{c}
 * configSTACK_DEPTH_TYPE uxTaskGetStackHighWaterMark2( TaskHandle_t xTask );
 * @endcode
 *
 * INCLUDE_uxTaskGetStackHighWaterMark2 must be set to 1 in FreeRTOSConfig.h for
 * this function to be available.
 *
 * Returns the high water mark of the stack associated with xTask.  That is,
 * the minimum free stack space there has been (in words, so on a 32 bit machine
 * a value of 1 means 4 bytes) since the task started.  The smaller the returned
 * number the closer the task has come to overflowing its stack.
 *
 * uxTaskGetStackHighWaterMark() and uxTaskGetStackHighWaterMark2() are the
 * same except for their return type.  Using configSTACK_DEPTH_TYPE allows the
 * user to determine the return type.  It gets around the problem of the value
 * overflowing on 8-bit types without breaking backward compatibility for
 * applications that expect an 8-bit return type.
 *
 * @param xTask Handle of the task associated with the stack to be checked.
 * Set xTask to NULL to check the stack of the calling task.
 *
 * @return The smallest amount of free stack space there has been (in words, so
 * actual spaces on the stack rather than bytes) since the task referenced by
 * xTask was created.
 */
configSTACK_DEPTH_TYPE uxTaskGetStackHighWaterMark2( TaskHandle_t xTask );

/* When using trace macros it is sometimes necessary to include task.h before
 * FreeRTOS.h.  When this is done TaskHookFunction_t will not yet have been defined,
 * so the following two prototypes will cause a compilation error.  This can be
 * fixed by simply guarding against the inclusion of these two prototypes unless
 * they are explicitly required by the configUSE_APPLICATION_TASK_TAG configuration
 * constant. */
#ifdef configUSE_APPLICATION_TASK_TAG
    #if configUSE_APPLICATION_TASK_TAG == 1

/**
 * task.h
 * @code{c}
 * void vTaskSetApplicationTaskTag( TaskHandle_t xTask, TaskHookFunction_t pxHookFunction );
 * @endcode
 *
 * Sets pxHookFunction to be the task hook function used by the task xTask.
 * Passing xTask as NULL has the effect of setting the calling tasks hook
 * function.
 */
        void vTaskSetApplicationTaskTag( TaskHandle_t xTask,
                                         TaskHookFunction_t pxHookFunction );

/**
 * task.h
 * @code{c}
 * void xTaskGetApplicationTaskTag( TaskHandle_t xTask );
 * @endcode
 *
 * Returns the pxHookFunction value assigned to the task xTask.  Do not
 * call from an interrupt service routine - call
 * xTaskGetApplicationTaskTagFromISR() instead.
 */
        TaskHookFunction_t xTaskGetApplicationTaskTag( TaskHandle_t xTask );

/**
 * task.h
 * @code{c}
 * void xTaskGetApplicationTaskTagFromISR( TaskHandle_t xTask );
 * @endcode
 *
 * Returns the pxHookFunction value assigned to the task xTask.  Can
 * be called from an interrupt service routine.
 */
        TaskHookFunction_t xTaskGetApplicationTaskTagFromISR( TaskHandle_t xTask );
    #endif /* configUSE_APPLICATION_TASK_TAG ==1 */
#endif /* ifdef configUSE_APPLICATION_TASK_TAG */

/**
 * task.h
 * @code{c}
 * BaseType_t xTaskCallApplicationTaskHook( TaskHandle_t xTask, void *pvParameter );
 * @endcode
 *
 * Calls the hook function associated with xTask.  Passing xTask as NULL has
 * the effect of calling the Running tasks (the calling task) hook function.
 *
 * pvParameter is passed to the hook function for the task to interpret as it
 * wants.  The return value is the value returned by the task hook function
 * registered by the user.
 */
BaseType_t xTaskCallApplicationTaskHook( TaskHandle_t xTask,
                                         void * pvParameter );

/**
 * xTaskGetIdleTaskHandle() is only available if
 * INCLUDE_xTaskGetIdleTaskHandle is set to 1 in FreeRTOSConfig.h.
 *
 * Simply returns the handle of the idle task.  It is not valid to call
 * xTaskGetIdleTaskHandle() before the scheduler has been started.
 */
TaskHandle_t xTaskGetIdleTaskHandle( void );

/*
 * Return the handle of the calling task.
 */
TaskHandle_t xTaskGetCurrentTaskHandle( void );

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */
#endif /* INC_TASK_H */
