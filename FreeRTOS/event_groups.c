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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

typedef struct EventGroupDef_t
{
    struct rt_event event;
} EventGroup_t;

static volatile rt_uint8_t event_index = 0;

/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    EventGroupHandle_t xEventGroupCreateStatic( StaticEventGroup_t * pxEventGroupBuffer )
    {
        char name[RT_NAME_MAX] = {0};

        /* A StaticEventGroup_t object must be provided. */
        configASSERT( pxEventGroupBuffer );

        rt_snprintf( name, RT_NAME_MAX, "event%02d", event_index++ );
        rt_event_init( ( rt_event_t ) pxEventGroupBuffer, name, RT_IPC_FLAG_PRIO );

        return ( EventGroupHandle_t ) pxEventGroupBuffer;
    }

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

    EventGroupHandle_t xEventGroupCreate( void )
    {
        EventGroup_t * pxEventBits;
        char name[RT_NAME_MAX] = {0};

        rt_snprintf( name, RT_NAME_MAX, "event%02d", event_index++ );
        pxEventBits = ( EventGroup_t * ) rt_event_create( name, RT_IPC_FLAG_PRIO );

        return pxEventBits;
    }

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

EventBits_t xEventGroupWaitBits( EventGroupHandle_t xEventGroup,
                                 const EventBits_t uxBitsToWaitFor,
                                 const BaseType_t xClearOnExit,
                                 const BaseType_t xWaitForAllBits,
                                 TickType_t xTicksToWait )
{
    rt_event_t event = ( rt_event_t ) xEventGroup;
    rt_uint8_t option = 0;
    rt_uint32_t recved;
    rt_base_t level;
    rt_err_t err;

    /* Check the user is not attempting to wait on the bits used by the kernel
     * itself, and that at least one bit is being requested. */
    configASSERT( xEventGroup );
    configASSERT( uxBitsToWaitFor != 0 );
    #if ( INCLUDE_xTaskGetSchedulerState == 1 )
        {
            configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) && ( xTicksToWait != 0 ) ) );
        }
    #endif

    if ( xWaitForAllBits != pdFALSE )
    {
        option |= RT_EVENT_FLAG_AND;
    }
    else
    {
        option |= RT_EVENT_FLAG_OR;
    }
    if ( xClearOnExit != pdFALSE )
    {
        option |= RT_EVENT_FLAG_CLEAR;
    }
    err = rt_event_recv( event, ( rt_uint32_t ) uxBitsToWaitFor, option, ( rt_int32_t ) xTicksToWait, &recved );

    if ( err != RT_EOK )
    {
        level = rt_hw_interrupt_disable();
        recved = event->set;
        rt_hw_interrupt_enable(level);
    }

    return ( EventBits_t ) recved;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupClearBits( EventGroupHandle_t xEventGroup,
                                  const EventBits_t uxBitsToClear )
{
    rt_event_t event = ( rt_event_t ) xEventGroup;
    EventBits_t uxReturn;
    rt_base_t level;

    configASSERT( xEventGroup );

    level = rt_hw_interrupt_disable();
    uxReturn = ( EventBits_t ) event->set;
    event->set &= ~( ( rt_uint32_t ) uxBitsToClear );
    rt_hw_interrupt_enable( level );

    return uxReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xEventGroupClearBitsFromISR( EventGroupHandle_t xEventGroup,
                                        const EventBits_t uxBitsToClear )
{
    return xEventGroupClearBits( xEventGroup, uxBitsToClear );
}

EventBits_t xEventGroupGetBitsFromISR( EventGroupHandle_t xEventGroup )
{
    rt_event_t event = ( rt_event_t ) xEventGroup;
    EventBits_t uxReturn;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    uxReturn = ( EventBits_t ) event->set;
    rt_hw_interrupt_enable( level );

    return uxReturn;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupSetBits( EventGroupHandle_t xEventGroup,
                                const EventBits_t uxBitsToSet )
{
    rt_event_t event = ( rt_event_t ) xEventGroup;
    rt_base_t level;
    EventBits_t uxReturn;

    configASSERT( xEventGroup );

    rt_event_send( event, ( rt_uint32_t ) uxBitsToSet);

    level = rt_hw_interrupt_disable();
    uxReturn = ( EventBits_t ) event->set;
    rt_hw_interrupt_enable(level);

    return uxReturn;
}
/*-----------------------------------------------------------*/

void vEventGroupDelete( EventGroupHandle_t xEventGroup )
{
    rt_event_t event = ( rt_event_t ) xEventGroup;

    configASSERT( xEventGroup );

#if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
    if ( rt_object_is_systemobject( ( rt_object_t ) event ) )
#endif
    {
    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        rt_event_detach( event );
    #endif
#if ( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
    }
    else
    {
#endif
    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
        rt_event_delete( event );
    #endif
    }
}
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) )

    BaseType_t xEventGroupSetBitsFromISR( EventGroupHandle_t xEventGroup,
                                          const EventBits_t uxBitsToSet,
                                          BaseType_t * pxHigherPriorityTaskWoken )
    {
        xEventGroupSetBits( xEventGroup, uxBitsToSet );
        if ( pxHigherPriorityTaskWoken != NULL)
        {
            pxHigherPriorityTaskWoken = pdFALSE;
        }

        return pdPASS;
    }

#endif /* if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) ) */
/*-----------------------------------------------------------*/
