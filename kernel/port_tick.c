/*
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "mt2625.h"
#include "hal.h"
#include "timers.h"
#ifdef MTK_NBIOT_TARGET_BUILD
#include "gkitimer.h"
#endif

#if configUSE_TICKLESS_IDLE == 2
#include "task.h"
#include "memory_attribute.h"
#include "hal_sleep_manager.h"
#include "hal_sleep_manager_internal.h"
#include "hal_sleep_manager_platform.h"
#include "core_cm4.h"
#include "hal_gpt.h"
#include "hal_rtc.h"
#include "hal_rtc_internal.h"
#include "hal_log.h"
#include "hal_eint.h"
#include "hal_wdt.h"

#ifdef MTK_NVDM_ENABLE
#include "nvdm.h"
#include "memory_map.h"
#endif

#ifdef MTK_NBIOT_TARGET_BUILD
#include "psc_api.h"
#endif
#ifdef MTK_SWLA_ENABLE
#include "swla.h"
#endif
#endif /* configUSE_TICKLESS_IDLE == 2 */

#include "rtthread.h"
#include "rthw.h"

//#define TICKLESS_DEEBUG_ENABLE
#ifdef  TICKLESS_DEEBUG_ENABLE
#define log_debug(_message,...) printf(_message, ##__VA_ARGS__)
#else
#define log_debug(_message,...)
#endif

#ifdef MTK_NBIOT_TARGET_BUILD
#ifdef PSC_WORST_CASE_WAKE_UP_TIME_MS
#define MT2625_DEEPER_SLEEP_OVERHEAD    PSC_WORST_CASE_WAKE_UP_TIME_MS //ms
#define MT2625_DEEP_SLEEP_OVERHEAD      PSC_WORST_CASE_WAKE_UP_TIME_MS
#else
#define MT2625_DEEPER_SLEEP_OVERHEAD    780
#define MT2625_DEEP_SLEEP_OVERHEAD      770     //ms
#endif
#else
#define MT2625_DEEPER_SLEEP_OVERHEAD    780
#define MT2625_DEEP_SLEEP_OVERHEAD      770     //ms
#endif /* MTK_NBIOT_TARGET_BUILD */

#define MT2625_DEEPER_SLEEP_CRITERIA    10800000//ms
#define MT2625_DEEP_SLEEP_CRITERIA      11000   //ms
#define MT2625_LIGHT_SLEEP_OVERHEAD     8       //ms
#define MT2625_LIGHT_SLEEP_CRITERIA     16      //ms
#define MT2625_IDLE_SLEEP_OVERHEAD      1       //ms

#if MT2625_DEEP_SLEEP_OVERHEAD >= MT2625_DEEP_SLEEP_CRITERIA
#error "deep sleep overhead is too long"
#endif

#if MT2625_DEEP_SLEEP_OVERHEAD < 770
#error "deep sleep overhead is too short"
#endif

GPT_REGISTER_T *os_gpt0 = OS_GPT0;

#if configUSE_TICKLESS_IDLE != 0
/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
static uint32_t xMaximumPossibleSuppressedTicks = 0;

static unsigned int ds_set_time_fail = 0;
static unsigned int ls_set_time_fail = 0;

static unsigned int ds_no_rtc_alarm = 0;
static unsigned int ds_rtc_too_short = 0;
static unsigned int ds_psc_too_short = 0;
static unsigned int ls_sys_too_short = 0;
static unsigned int ls_psc_too_short = 0;

ATTR_RWDATA_IN_RETSRAM uint32_t systick_backup;

extern uint32_t hal_sleep_manager_sleep_driver_dump_handle_name(void);
#endif /* configUSE_TICKLESS_IDLE != 0 */

bool reset_gpt_to_systick = false;

uint32_t os_gpt0_get_count(void)
{
    return os_gpt0->GPT_COUNT;
}

void os_gpt0_pause(void)
{
    os_gpt0->GPT_CON_UNION.GPT_CON &= 0xFFFFFFFE;
}

//clear OS GPT0 count, it can only be invoked when GPTO0 is disabled.
void os_gpt0_clear_count(void)
{
   os_gpt0->GPT_CLR = 0x01;
   while(os_gpt0->GPT_COUNT);
}

void os_gpt0_resume(bool update, uint32_t new_compare)
{
    if (update) {
        reset_gpt_to_systick = true;
        os_gpt0->GPT_COMPARE = new_compare;
    }
    os_gpt0->GPT_CON_UNION.GPT_CON |= 0x00000001;
}

#if configUSE_TICKLESS_IDLE == 2
static uint32_t ticklessTryCount  = 0;
static uint32_t ticklessDPSCount  = 0;
static uint32_t ticklessDSCount   = 0;
static uint32_t ticklessLSCount   = 0;
static uint32_t ticklessIdleCount = 0;

static uint32_t TimeStampSystick, TimeStampCounter;

static bool sys_show_lock = false;
static uint32_t sys_show_freq = 0;

void tickless_log_timestamp(void);
TimerHandle_t timestamp_timer = NULL;

#define DEEP_SLEEP_HW_WAKEUP_TIME 2
#define DEEP_SLEEP_SW_BACKUP_RESTORE_TIME 8
extern uint32_t cli_dtim_sleep_mode;

#ifdef MTK_MINI_DUMP_ENABLE
extern void MiniDump_RecordEnterLightSleepInfo(void);
extern void MiniDump_RecordExitLightSleepInfo(uint32_t SleepTime);
#endif /*MTK_MINI_DUMP_ENABLE*/

/* RTC 32.768KHz Freq*/
float RTC_Freq = 32.768f;
#define MinimumIdleTime 2  //tick(20ms in total)

bool tryDeepSleep(void);
bool tryLightSleep(uint32_t sleep_time);
void doSystickCalibration(uint32_t maxSystickCompensation);

void systick_backup_callback(void* data, uint32_t mode)
{
    systick_backup = (uint32_t)xTaskGetTickCount();
}

bool tryDeepSleep(void)
{
    uint32_t sleep_time = 0;
    uint32_t rtc_sleep_time = 0;
    bool use_rtc_time = true;
    hal_sleep_manager_status_t ret;
    hal_rtc_status_t rtc_ret;

#ifdef MTK_NBIOT_TARGET_BUILD
    uint32_t psc_sleep_time = 0;
    psc_sleep_time = psc_get_sleep_opportunity_duration();
#endif
    rtc_ret = rtc_sw_get_remaining_time(&rtc_sleep_time);

#ifdef MTK_NBIOT_TARGET_BUILD
    if (rtc_ret == HAL_RTC_STATUS_ERROR) {
        sleep_time = psc_sleep_time;
        use_rtc_time = false;
        ds_no_rtc_alarm++;
    } else {
        rtc_sleep_time = (float)rtc_sleep_time * 62.5;
        if (psc_sleep_time > rtc_sleep_time) {
            sleep_time = rtc_sleep_time;
        } else {
            sleep_time = psc_sleep_time;
            use_rtc_time = false;
        }
    }
#else
    if (rtc_ret == RTC_SW_TIMER_STATUS_ERROR) {
        return false;
    } else {
        sleep_time = rtc_sleep_time;
    }
#endif

    if (sleep_time >= MT2625_DEEPER_SLEEP_CRITERIA) {
        sleep_time -= MT2625_DEEPER_SLEEP_OVERHEAD;
        sleep_manager_set_sleep_mode(HAL_SLEEP_MODE_DEEPER_SLEEP);
        ticklessDPSCount++;
    } else if (sleep_time >= MT2625_DEEP_SLEEP_CRITERIA) {
        sleep_time -= MT2625_DEEP_SLEEP_OVERHEAD;
        sleep_manager_set_sleep_mode(HAL_SLEEP_MODE_DEEP_SLEEP);
        ticklessDSCount++;
    } else {
        // sleep_time not long enough
        if (use_rtc_time) {
            ds_rtc_too_short++;
        } else {
            ds_psc_too_short++;
        }
        return false;
    }

    ret = hal_sleep_manager_set_sleep_time(sleep_time);
    if (ret == HAL_SLEEP_MANAGER_ERROR) {
        ds_set_time_fail++;
        return false;
    }

    sleep_management_backup_systime((uint32_t)xTaskGetTickCount() * (1000 / configTICK_RATE_HZ));
    hal_sleep_manager_enter_sleep_mode(sleep_manager_get_sleep_mode());

    // Never be here...
    assert(0);

    return true;
}

bool tryLightSleep(uint32_t try_sleep_ms)
{
    uint32_t sleep_time = try_sleep_ms;
    bool use_psc_time = false;
    hal_sleep_manager_status_t ret;

#ifdef MTK_NBIOT_TARGET_BUILD
    uint32_t psc_sleep_time = 0;

    psc_sleep_time = psc_get_sleep_opportunity_duration();
    if (try_sleep_ms > psc_sleep_time) {
        sleep_time = psc_sleep_time;
        use_psc_time = true;
    }
#endif

    if (sleep_time >= MT2625_LIGHT_SLEEP_CRITERIA) {
        sleep_time -= MT2625_LIGHT_SLEEP_OVERHEAD;
        sleep_manager_set_sleep_mode(HAL_SLEEP_MODE_LIGHT_SLEEP);
        ticklessLSCount++;
    } else {
        if (use_psc_time) {
            ls_psc_too_short++;
        } else {
            ls_sys_too_short++;
        }
        return false;
    }

    ret = hal_sleep_manager_set_sleep_time(sleep_time);
    if (ret == HAL_SLEEP_MANAGER_ERROR) {
        // __asm volatile("cpsie i");
        // os_gpt0_resume(false, 0);

        ls_set_time_fail++;

        return false;
    }

#ifdef MTK_MINI_DUMP_ENABLE
    //Mini dump record the system enter the light sleep
    MiniDump_RecordEnterLightSleepInfo();
#endif

#ifdef MTK_SWLA_ENABLE
    SLA_CustomLogging("SMLS", SA_START);
#endif
    hal_sleep_manager_enter_sleep_mode(sleep_manager_get_sleep_mode());
#ifdef MTK_SWLA_ENABLE
    SLA_CustomLogging("SMLS", SA_STOP);
#endif

    doSystickCalibration(try_sleep_ms / (1000 / configTICK_RATE_HZ));

    return true;
}

void doSystickCalibration(uint32_t maxSystickCompensation)
{
    uint32_t nowCount, nowTick;
    uint32_t ulCompleteSystickPeriods, ulCompletedCountDecrements, ulCompletedTickDecrements;

    // Calculate time(systick) to jump
    hal_gpt_get_free_run_count(HAL_GPT_CLOCK_SOURCE_32K, &nowCount);
    nowTick = ((uint32_t)xTaskGetTickCount() * (1000 / configTICK_RATE_HZ));

    // get counter distance from last record
    if (nowCount >= TimeStampCounter)
    {
        ulCompletedCountDecrements = nowCount - TimeStampCounter;
    }
    else
    {
        ulCompletedCountDecrements = nowCount + (0xFFFFFFFF - TimeStampCounter);
    }

    // get systick distance from last record
    if (nowTick >= TimeStampSystick)
    {
        ulCompletedTickDecrements = nowTick - TimeStampSystick;
    }
    else
    {
        ulCompletedTickDecrements = nowTick + (0xFFFFFFFF - TimeStampSystick);
    }

    // get counter distance for this sleep
    ulCompletedCountDecrements = (unsigned int)(((float)ulCompletedCountDecrements) - ((float)ulCompletedTickDecrements * RTC_Freq));
    // calculate ticks for jumpping
    ulCompleteSystickPeriods = ((unsigned int)(((float)ulCompletedCountDecrements) / RTC_Freq)) / ((1000 / configTICK_RATE_HZ));

    //Limit OS Tick Compensation Value
    if (ulCompleteSystickPeriods > (maxSystickCompensation - 1))
    {
        ulCompleteSystickPeriods = maxSystickCompensation - 1;
    }

#ifdef MTK_MINI_DUMP_ENABLE
    //Mini dump record the system exit the light sleep
    MiniDump_RecordExitLightSleepInfo(ulCompleteSystickPeriods);
#endif

    vTaskStepTick(ulCompleteSystickPeriods);
#ifdef MTK_NBIOT_TARGET_BUILD
    extern void RerecordGptCnt(void);
    RerecordGptCnt();
    KiTick(ulCompleteSystickPeriods);
#endif
}

bool AST_vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    TickType_t xModifiableIdleTime;
    volatile static unsigned int ulAST_Reload_ms;

    // __asm volatile("cpsid i");
    rt_base_t level = rt_hw_interrupt_disable();
    os_gpt0_pause();

#if 0 /* Not used for rt-thread */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
        os_gpt0_resume(false, 0);
        // __asm volatile("cpsie i");
        rt_hw_interrupt_enable(level);

        AST_abort++;

        return false;
    }
#endif

#ifdef MTK_SYSTEM_HANG_CHECK_ENABLE
    hal_wdt_disable(HAL_WDT_DISABLE_MAGIC);
#endif

    // transfer expected idle tick to ms
    ulAST_Reload_ms = (xExpectedIdleTime - 1) * (1000 / configTICK_RATE_HZ);

    xModifiableIdleTime = xExpectedIdleTime;

    configPRE_SLEEP_PROCESSING(xModifiableIdleTime);
    if (xModifiableIdleTime > 0)
    {
        if (hal_sleep_manager_is_sleep_lock_locked(HAL_SLEEP_LOCK_DEEP) == false)
        {
            tryDeepSleep();
        }

        // try deep sleep failed, try light sleep
        if (tryLightSleep(ulAST_Reload_ms) == false)
        {
            os_gpt0_resume(false, 0);
            rt_hw_interrupt_enable(level);

            #ifdef MTK_SYSTEM_HANG_CHECK_ENABLE
                hal_wdt_enable(HAL_WDT_ENABLE_MAGIC);
            #endif
            return false;
        }
    }
    configPOST_SLEEP_PROCESSING(xExpectedIdleTime);

    os_gpt0_resume(false, 0);
    rt_hw_interrupt_enable(level);

#ifdef MTK_SYSTEM_HANG_CHECK_ENABLE
    hal_wdt_enable(HAL_WDT_ENABLE_MAGIC);
#endif

    // light sleep succeeded
    return true;
}

void tickless_handler(uint32_t xExpectedIdleTime)
{
    uint32_t lock_status = hal_sleep_manager_is_sleep_lock_locked(HAL_SLEEP_LOCK_ALL);

    ticklessTryCount++;

    if ((xExpectedIdleTime > MinimumIdleTime) && (lock_status == 0)) {
        if (AST_vPortSuppressTicksAndSleep(xExpectedIdleTime)) {
            // sleep success
            return;
        }
    } else {
        //all_sleep_locked++;
    }

    // Sleep fail, goto idle
    ticklessIdleCount++;
    hal_sleep_manager_enter_sleep_mode(HAL_SLEEP_MODE_IDLE);
}

static void tickless_log_timestamp_callback(TimerHandle_t expiredTimer)
{
    tickless_log_timestamp();
}

void tickless_system_status()
{}

void tickless_log_timestamp()
{
    //uint32_t rtc_clock;
    hal_gpt_get_free_run_count(HAL_GPT_CLOCK_SOURCE_32K, &TimeStampCounter);
    TimeStampSystick = (uint32_t)xTaskGetTickCount() * (1000 / configTICK_RATE_HZ);
}
#endif /* configUSE_TICKLESS_IDLE == 2 */

void os_gpt_interrupt_handle(hal_nvic_irq_t irq_number)
{
    os_gpt0->GPT_IRQ_ACK = 0x01;
    /* Run GKI tick handler at here */

#if configUSE_TICKLESS_IDLE == 2
    if (reset_gpt_to_systick) {
        reset_gpt_to_systick = false;
        os_gpt0->GPT_COMPARE = portTICK_PERIOD_MS * 1000;
    }
#endif

#ifdef MTK_NBIOT_TARGET_BUILD
    extern void KiTick(KernelTicks increment);
    extern uint32_t SysInitStatus_Query(void);
    if (SysInitStatus_Query())
        KiTick(1);
#endif

    /* Run FreeRTOS tick handler*/
    extern void SysTick_Handler(void);
    SysTick_Handler();
}

void os_gpt_init(uint32_t ms)
{
    // PDN???
    /* set 13 divided with 13M source */
    os_gpt0->GPT_CON_UNION.GPT_CON |= (1 << 16);   // disable clock before config
    os_gpt0->GPT_CLK = 0xc;
    os_gpt0->GPT_CON_UNION.GPT_CON &= ~(1 << 16);   // enable clock
    os_gpt0->GPT_COMPARE = ms * 1000;

    /* clear */
    os_gpt0->GPT_CLR = 0x01;
    while (os_gpt0->GPT_COUNT);

    /* enable IRQ */
    os_gpt0->GPT_IRQ_EN = 0x1;
    /* enable GPT0 clk and repeat mode and enable GPT0 */
    os_gpt0->GPT_CON_UNION.GPT_CON |= 0x101;

    /* register and enable IRQ */
    hal_nvic_register_isr_handler(OS_GPT_IRQn, (hal_nvic_isr_t)os_gpt_interrupt_handle);
    NVIC_EnableIRQ(OS_GPT_IRQn);
}

static void vport_tickless_config(void)
{
#if configUSE_TICKLESS_IDLE == 2
    nvdm_status_t status;
    uint32_t read_size = 1;

    status = nvdm_read_data_item("sleep_manager", "sys_show_lock", &sys_show_lock, &read_size);
    read_size = 4;
    status = nvdm_read_data_item("sleep_manager", "sys_show_freq", (uint8_t*)&sys_show_freq, &read_size);

    if (status == NVDM_STATUS_OK) {
        rt_kprintf("sys_show_lock = %d\r\n", sys_show_lock);
        rt_kprintf("sys_show_freq = %u\r\n", (unsigned int)sys_show_freq);
    } else {
        rt_kprintf("read sys_show_lock and sys_show_freq from NVDM fail\r\n");
    }

    sleep_management_register_suspend_callback(SLEEP_BACKUP_RESTORE_USER,
                                               systick_backup_callback,
                                               NULL,
                                               HAL_SLEEP_MODE_DEEP_SLEEP|HAL_SLEEP_MODE_DEEPER_SLEEP);
#endif /* configUSE_TICKLESS_IDLE == 2 */
}

void vPortSetupTimerInterrupt(void)
{
#if configUSE_TICKLESS_IDLE == 2
    static int init_timestamp = 0;
    if (init_timestamp == 0) {
        init_timestamp = 1;
        tickless_log_timestamp();

        timestamp_timer = xTimerCreate("timestamp_timer",
                                       1000 * 60 * 60 * 12 / portTICK_PERIOD_MS, //12hours
                                       true,
                                       NULL,
                                       tickless_log_timestamp_callback);
        if (timestamp_timer == NULL) {
            printf("timestamp_timer create fail\n");
        } else {
            if (xTimerStart(timestamp_timer, 0) != pdPASS) {
                printf("timestamp_timer xTimerStart fail\n");
            }
        }
    }
#endif /* configUSE_TICKLESS_IDLE */

#if configUSE_TICKLESS_IDLE != 0
    /* Calculate the constants required to configure the tick interrupt. */
    {
        xMaximumPossibleSuppressedTicks = 0xFFFFFFFF / (portTICK_PERIOD_MS * 1000);
    }
#endif /* configUSE_TICKLESS_IDLE  != 0*/

/**
 * move 'os_gpt_init' to drv_common.c, will exec before 'rt_sheduler_start'. 
 * Designed to provide a system clock base os gpt timer.
*/
#if 0 /* move to drv_common.c by rt-thread */
#ifdef TEMP_FPGA_ML1S_HACKS
/* Tick has to be 20 times slower when using slow bitfiles */
extern bool use_slow_bitfile;
if (use_slow_bitfile==true)
    os_gpt_init(portTICK_PERIOD_MS*20); /* 1tick = 10ms */
else
#endif //TEMP_FPGA_ML1S_HACKS
    os_gpt_init(portTICK_PERIOD_MS); /* 1tick = 10ms */
#endif /* 0 */
    vport_tickless_config();
}
