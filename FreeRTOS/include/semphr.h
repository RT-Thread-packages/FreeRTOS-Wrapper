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

#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#ifndef INC_FREERTOS_H
    #error "include FreeRTOS.h" must appear in source files before "include semphr.h"
#endif

#include "queue.h"

typedef QueueHandle_t SemaphoreHandle_t;

/**
 * semphr. h
 * @code{c}
 * xSemaphoreTakeRecursive(
 *                          SemaphoreHandle_t xMutex,
 *                          TickType_t xBlockTime
 *                        );
 * @endcode
 *
 * <i>Macro</i> to recursively obtain, or 'take', a mutex type semaphore.
 * The mutex must have previously been created using a call to
 * xSemaphoreCreateRecursiveMutex();
 *
 * configUSE_RECURSIVE_MUTEXES must be set to 1 in FreeRTOSConfig.h for this
 * macro to be available.
 *
 * This macro must not be used on mutexes created using xSemaphoreCreateMutex().
 *
 * A mutex used recursively can be 'taken' repeatedly by the owner. The mutex
 * doesn't become available again until the owner has called
 * xSemaphoreGiveRecursive() for each successful 'take' request.  For example,
 * if a task successfully 'takes' the same mutex 5 times then the mutex will
 * not be available to any other task until it has also  'given' the mutex back
 * exactly five times.
 *
 * @param xMutex A handle to the mutex being obtained.  This is the
 * handle returned by xSemaphoreCreateRecursiveMutex();
 *
 * @param xBlockTime The time in ticks to wait for the semaphore to become
 * available.  The macro portTICK_PERIOD_MS can be used to convert this to a
 * real time.  A block time of zero can be used to poll the semaphore.  If
 * the task already owns the semaphore then xSemaphoreTakeRecursive() will
 * return immediately no matter what the value of xBlockTime.
 *
 * @return pdTRUE if the semaphore was obtained.  pdFALSE if xBlockTime
 * expired without the semaphore becoming available.
 *
 * Example usage:
 * @code{c}
 * SemaphoreHandle_t xMutex = NULL;
 *
 * // A task that creates a mutex.
 * void vATask( void * pvParameters )
 * {
 *  // Create the mutex to guard a shared resource.
 *  xMutex = xSemaphoreCreateRecursiveMutex();
 * }
 *
 * // A task that uses the mutex.
 * void vAnotherTask( void * pvParameters )
 * {
 *  // ... Do other things.
 *
 *  if( xMutex != NULL )
 *  {
 *      // See if we can obtain the mutex.  If the mutex is not available
 *      // wait 10 ticks to see if it becomes free.
 *      if( xSemaphoreTakeRecursive( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
 *      {
 *          // We were able to obtain the mutex and can now access the
 *          // shared resource.
 *
 *          // ...
 *          // For some reason due to the nature of the code further calls to
 *          // xSemaphoreTakeRecursive() are made on the same mutex.  In real
 *          // code these would not be just sequential calls as this would make
 *          // no sense.  Instead the calls are likely to be buried inside
 *          // a more complex call structure.
 *          xSemaphoreTakeRecursive( xMutex, ( TickType_t ) 10 );
 *          xSemaphoreTakeRecursive( xMutex, ( TickType_t ) 10 );
 *
 *          // The mutex has now been 'taken' three times, so will not be
 *          // available to another task until it has also been given back
 *          // three times.  Again it is unlikely that real code would have
 *          // these calls sequentially, but instead buried in a more complex
 *          // call structure.  This is just for illustrative purposes.
 *          xSemaphoreGiveRecursive( xMutex );
 *          xSemaphoreGiveRecursive( xMutex );
 *          xSemaphoreGiveRecursive( xMutex );
 *
 *          // Now the mutex can be taken by other tasks.
 *      }
 *      else
 *      {
 *          // We could not obtain the mutex and can therefore not access
 *          // the shared resource safely.
 *      }
 *  }
 * }
 * @endcode
 * \defgroup xSemaphoreTakeRecursive xSemaphoreTakeRecursive
 * \ingroup Semaphores
 */
#if ( configUSE_RECURSIVE_MUTEXES == 1 )
    #define xSemaphoreTakeRecursive( xMutex, xBlockTime )    xQueueTakeMutexRecursive( ( xMutex ), ( xBlockTime ) )
#endif

/**
 * semphr. h
 * @code{c}
 * xSemaphoreGiveRecursive( SemaphoreHandle_t xMutex );
 * @endcode
 *
 * <i>Macro</i> to recursively release, or 'give', a mutex type semaphore.
 * The mutex must have previously been created using a call to
 * xSemaphoreCreateRecursiveMutex();
 *
 * configUSE_RECURSIVE_MUTEXES must be set to 1 in FreeRTOSConfig.h for this
 * macro to be available.
 *
 * This macro must not be used on mutexes created using xSemaphoreCreateMutex().
 *
 * A mutex used recursively can be 'taken' repeatedly by the owner. The mutex
 * doesn't become available again until the owner has called
 * xSemaphoreGiveRecursive() for each successful 'take' request.  For example,
 * if a task successfully 'takes' the same mutex 5 times then the mutex will
 * not be available to any other task until it has also  'given' the mutex back
 * exactly five times.
 *
 * @param xMutex A handle to the mutex being released, or 'given'.  This is the
 * handle returned by xSemaphoreCreateMutex();
 *
 * @return pdTRUE if the semaphore was given.
 *
 * Example usage:
 * @code{c}
 * SemaphoreHandle_t xMutex = NULL;
 *
 * // A task that creates a mutex.
 * void vATask( void * pvParameters )
 * {
 *  // Create the mutex to guard a shared resource.
 *  xMutex = xSemaphoreCreateRecursiveMutex();
 * }
 *
 * // A task that uses the mutex.
 * void vAnotherTask( void * pvParameters )
 * {
 *  // ... Do other things.
 *
 *  if( xMutex != NULL )
 *  {
 *      // See if we can obtain the mutex.  If the mutex is not available
 *      // wait 10 ticks to see if it becomes free.
 *      if( xSemaphoreTakeRecursive( xMutex, ( TickType_t ) 10 ) == pdTRUE )
 *      {
 *          // We were able to obtain the mutex and can now access the
 *          // shared resource.
 *
 *          // ...
 *          // For some reason due to the nature of the code further calls to
 *          // xSemaphoreTakeRecursive() are made on the same mutex.  In real
 *          // code these would not be just sequential calls as this would make
 *          // no sense.  Instead the calls are likely to be buried inside
 *          // a more complex call structure.
 *          xSemaphoreTakeRecursive( xMutex, ( TickType_t ) 10 );
 *          xSemaphoreTakeRecursive( xMutex, ( TickType_t ) 10 );
 *
 *          // The mutex has now been 'taken' three times, so will not be
 *          // available to another task until it has also been given back
 *          // three times.  Again it is unlikely that real code would have
 *          // these calls sequentially, it would be more likely that the calls
 *          // to xSemaphoreGiveRecursive() would be called as a call stack
 *          // unwound.  This is just for demonstrative purposes.
 *          xSemaphoreGiveRecursive( xMutex );
 *          xSemaphoreGiveRecursive( xMutex );
 *          xSemaphoreGiveRecursive( xMutex );
 *
 *          // Now the mutex can be taken by other tasks.
 *      }
 *      else
 *      {
 *          // We could not obtain the mutex and can therefore not access
 *          // the shared resource safely.
 *      }
 *  }
 * }
 * @endcode
 * \defgroup xSemaphoreGiveRecursive xSemaphoreGiveRecursive
 * \ingroup Semaphores
 */
#if ( configUSE_RECURSIVE_MUTEXES == 1 )
    #define xSemaphoreGiveRecursive( xMutex )    xQueueGiveMutexRecursive( ( xMutex ) )
#endif

/**
 * semphr. h
 * @code{c}
 * SemaphoreHandle_t xSemaphoreCreateRecursiveMutex( void );
 * @endcode
 *
 * Creates a new recursive mutex type semaphore instance, and returns a handle
 * by which the new recursive mutex can be referenced.
 *
 * Internally, within the FreeRTOS implementation, recursive mutexs use a block
 * of memory, in which the mutex structure is stored.  If a recursive mutex is
 * created using xSemaphoreCreateRecursiveMutex() then the required memory is
 * automatically dynamically allocated inside the
 * xSemaphoreCreateRecursiveMutex() function.  (see
 * https://www.FreeRTOS.org/a00111.html).  If a recursive mutex is created using
 * xSemaphoreCreateRecursiveMutexStatic() then the application writer must
 * provide the memory that will get used by the mutex.
 * xSemaphoreCreateRecursiveMutexStatic() therefore allows a recursive mutex to
 * be created without using any dynamic memory allocation.
 *
 * Mutexes created using this macro can be accessed using the
 * xSemaphoreTakeRecursive() and xSemaphoreGiveRecursive() macros.  The
 * xSemaphoreTake() and xSemaphoreGive() macros must not be used.
 *
 * A mutex used recursively can be 'taken' repeatedly by the owner. The mutex
 * doesn't become available again until the owner has called
 * xSemaphoreGiveRecursive() for each successful 'take' request.  For example,
 * if a task successfully 'takes' the same mutex 5 times then the mutex will
 * not be available to any other task until it has also  'given' the mutex back
 * exactly five times.
 *
 * This type of semaphore uses a priority inheritance mechanism so a task
 * 'taking' a semaphore MUST ALWAYS 'give' the semaphore back once the
 * semaphore it is no longer required.
 *
 * Mutex type semaphores cannot be used from within interrupt service routines.
 *
 * See xSemaphoreCreateBinary() for an alternative implementation that can be
 * used for pure synchronisation (where one task or interrupt always 'gives' the
 * semaphore and another always 'takes' the semaphore) and from within interrupt
 * service routines.
 *
 * @return xSemaphore Handle to the created mutex semaphore.  Should be of type
 * SemaphoreHandle_t.
 *
 * Example usage:
 * @code{c}
 * SemaphoreHandle_t xSemaphore;
 *
 * void vATask( void * pvParameters )
 * {
 *  // Semaphore cannot be used before a call to xSemaphoreCreateMutex().
 *  // This is a macro so pass the variable in directly.
 *  xSemaphore = xSemaphoreCreateRecursiveMutex();
 *
 *  if( xSemaphore != NULL )
 *  {
 *      // The semaphore was created successfully.
 *      // The semaphore can now be used.
 *  }
 * }
 * @endcode
 * \defgroup xSemaphoreCreateRecursiveMutex xSemaphoreCreateRecursiveMutex
 * \ingroup Semaphores
 */
#if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configUSE_RECURSIVE_MUTEXES == 1 ) )
    #define xSemaphoreCreateRecursiveMutex()    xQueueCreateMutex( queueQUEUE_TYPE_RECURSIVE_MUTEX )
#endif

/**
 * semphr. h
 * @code{c}
 * SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic( StaticSemaphore_t *pxMutexBuffer );
 * @endcode
 *
 * Creates a new recursive mutex type semaphore instance, and returns a handle
 * by which the new recursive mutex can be referenced.
 *
 * Internally, within the FreeRTOS implementation, recursive mutexs use a block
 * of memory, in which the mutex structure is stored.  If a recursive mutex is
 * created using xSemaphoreCreateRecursiveMutex() then the required memory is
 * automatically dynamically allocated inside the
 * xSemaphoreCreateRecursiveMutex() function.  (see
 * https://www.FreeRTOS.org/a00111.html).  If a recursive mutex is created using
 * xSemaphoreCreateRecursiveMutexStatic() then the application writer must
 * provide the memory that will get used by the mutex.
 * xSemaphoreCreateRecursiveMutexStatic() therefore allows a recursive mutex to
 * be created without using any dynamic memory allocation.
 *
 * Mutexes created using this macro can be accessed using the
 * xSemaphoreTakeRecursive() and xSemaphoreGiveRecursive() macros.  The
 * xSemaphoreTake() and xSemaphoreGive() macros must not be used.
 *
 * A mutex used recursively can be 'taken' repeatedly by the owner. The mutex
 * doesn't become available again until the owner has called
 * xSemaphoreGiveRecursive() for each successful 'take' request.  For example,
 * if a task successfully 'takes' the same mutex 5 times then the mutex will
 * not be available to any other task until it has also  'given' the mutex back
 * exactly five times.
 *
 * This type of semaphore uses a priority inheritance mechanism so a task
 * 'taking' a semaphore MUST ALWAYS 'give' the semaphore back once the
 * semaphore it is no longer required.
 *
 * Mutex type semaphores cannot be used from within interrupt service routines.
 *
 * See xSemaphoreCreateBinary() for an alternative implementation that can be
 * used for pure synchronisation (where one task or interrupt always 'gives' the
 * semaphore and another always 'takes' the semaphore) and from within interrupt
 * service routines.
 *
 * @param pxMutexBuffer Must point to a variable of type StaticSemaphore_t,
 * which will then be used to hold the recursive mutex's data structure,
 * removing the need for the memory to be allocated dynamically.
 *
 * @return If the recursive mutex was successfully created then a handle to the
 * created recursive mutex is returned.  If pxMutexBuffer was NULL then NULL is
 * returned.
 *
 * Example usage:
 * @code{c}
 * SemaphoreHandle_t xSemaphore;
 * StaticSemaphore_t xMutexBuffer;
 *
 * void vATask( void * pvParameters )
 * {
 *  // A recursive semaphore cannot be used before it is created.  Here a
 *  // recursive mutex is created using xSemaphoreCreateRecursiveMutexStatic().
 *  // The address of xMutexBuffer is passed into the function, and will hold
 *  // the mutexes data structures - so no dynamic memory allocation will be
 *  // attempted.
 *  xSemaphore = xSemaphoreCreateRecursiveMutexStatic( &xMutexBuffer );
 *
 *  // As no dynamic memory allocation was performed, xSemaphore cannot be NULL,
 *  // so there is no need to check it.
 * }
 * @endcode
 * \defgroup xSemaphoreCreateRecursiveMutexStatic xSemaphoreCreateRecursiveMutexStatic
 * \ingroup Semaphores
 */
#if ( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configUSE_RECURSIVE_MUTEXES == 1 ) )
    #define xSemaphoreCreateRecursiveMutexStatic( pxStaticSemaphore )    xQueueCreateMutexStatic( queueQUEUE_TYPE_RECURSIVE_MUTEX, pxStaticSemaphore )
#endif /* configSUPPORT_STATIC_ALLOCATION */

#endif /* SEMAPHORE_H */
