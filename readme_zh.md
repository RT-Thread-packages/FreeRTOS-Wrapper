# RT-Thread操作系统的FreeRTOS兼容层
## FreeRTOS Application Compatibility Layer (ACL) for RT-Thread
## 让基于FreeRTOS开发的应用层无感地迁移到RT-Thread操作系统

中文 | [English](readme.md)

## 1 概述
这是一个针对RT-Thread国产操作系统的FreeRTOS操作系统兼容层，可以让原有基于FreeRTOS操作系统的项目快速、无感地迁移到RT-Thread操作系统上，实现在RT-Thread操作系统上无感的使用FreeRTOS的API，同时可以使用RT-Thread的丰富组件。项目基于FreeRTOS V10.4.6版本。目前已经支撑多款基于FreeRTOS编写的SDK落地RT-Thread。

### 1.1 RT-Thread的其他RTOS兼容层

- RT-Thread操作系统的μCOS-III兼容层：https://github.com/mysterywolf/RT-Thread-wrapper-of-uCOS-III
- RT-Thread操作系统的μCOS-II兼容层：https://github.com/mysterywolf/RT-Thread-wrapper-of-uCOS-II
- RT-Thread操作系统的RTX(即CMSIS-RTOS1)兼容层：https://github.com/RT-Thread-packages/CMSIS_RTOS1
- RT-Thread操作系统的RTX5(即CMSIS-RTOS2)兼容层：https://github.com/RT-Thread-packages/CMSIS_RTOS2
- RT-Thread操作系统的Arduino生态兼容层：https://github.com/RTduino/RTduino

## 2 FreeRTOS的API支持情况

兼容层对FreeRTOS API的支持情况如下：

### 2.1 任务控制
- [x] xTaskCreate
- [x] xTaskCreateStatic
- [ ] [xTaskCreateRestrictedStatic](https://www.freertos.org/xtaskcreaterestrictedstaticfreertos-mpu-specific.html)
- [x] vTaskDelete
- [x] vTaskDelay
- [x] vTaskDelayUntil
- [x] xTaskDelayUntil 
- [x] uxTaskPriorityGet
- [x] vTaskPrioritySet
- [x] vTaskSuspend (只支持挂起当前线程)
- [x] vTaskResume
- [x] xTaskResumeFromISR
- [x] xTaskAbortDelay
- [ ] [uxTaskGetSystemState](https://www.freertos.org/uxTaskGetSystemState.html)
- [ ] [vTaskGetInfo](https://www.freertos.org/vTaskGetInfo.html)
- [ ] [vTaskList](https://www.freertos.org/a00021.html#vTaskList)
- [ ] [vTaskGetRunTimeStats](https://www.freertos.org/a00021.html#vTaskGetRunTimeStats)
- [ ] [vTaskStartTrace](https://www.freertos.org/a00021.html#vTaskStartTrace)
- [ ] [ulTaskEndTrace](https://www.freertos.org/a00021.html#usTaskEndTrace)
- [ ] [SetThreadLocalStoragePointer](https://www.freertos.org/vTaskSetThreadLocalStoragePointer.html)
- [ ] [GetThreadLocalStoragePointer](https://www.freertos.org/pvTaskGetThreadLocalStoragePointer.html)
- [x] xTaskGetApplicationTaskTag 
- [x] xTaskGetCurrentTaskHandle
- [x] xTaskGetIdleTaskHandle
- [x] uxTaskGetStackHighWaterMark
- [x] eTaskGetState
- [x] pcTaskGetName
- [x] xTaskGetTickCount
- [x] xTaskGetTickCountFromISR
- [x] xTaskGetSchedulerState
- [x] uxTaskGetNumberOfTasks
- [x] vTaskSetApplicationTaskTag
- [x] xTaskCallApplicationTaskTag
- [x] vTaskSetTimeoutState
- [x] xTaskGetCheckForTimeout
### 2.2 内核控制
- [x] [taskYIELD](https://www.freertos.org/a00020.html#taskYIELD)
- [x] [taskENTER_CRITICAL](https://www.freertos.org/taskENTER_CRITICAL_taskEXIT_CRITICAL.html)
- [x] [taskEXIT_CRITICAL](https://www.freertos.org/taskENTER_CRITICAL_taskEXIT_CRITICAL.html)
- [x] [taskENTER_CRITICAL_FROM_ISR](https://www.freertos.org/taskENTER_CRITICAL_FROM_ISR_taskEXIT_CRITICAL_FROM_ISR.html)
- [x] [taskEXIT_CRITICAL_FROM_ISR](https://www.freertos.org/taskENTER_CRITICAL_FROM_ISR_taskEXIT_CRITICAL_FROM_ISR.html)
- [x] [taskDISABLE_INTERRUPTS](https://www.freertos.org/a00020.html#taskDISABLE_INTERRUPTS)
- [x] [taskENABLE_INTERRUPTS](https://www.freertos.org/a00020.html#taskENABLE_INTERRUPTS)
- [x] [vTaskStartScheduler](https://www.freertos.org/a00132.html)
- [x] [vTaskEndScheduler](https://www.freertos.org/a00133.html)
- [x] [vTaskSuspendAll](https://www.freertos.org/a00134.html)
- [x] [xTaskResumeAll](https://www.freertos.org/a00135.html)
- [ ] [vTaskStepTick](https://www.freertos.org/vTaskStepTick.html)
- [ ] [xTaskCatchUpTicks](https://www.freertos.org/xTaskCatchUpTicks.html)
### 2.3 任务通知
- [x] [xTaskNotifyGive](https://www.freertos.org/xTaskNotifyGive.html)
- [x] [vTaskNotifyGiveFromISR](https://www.freertos.org/vTaskNotifyGiveFromISR.html)
- [x] [ulTaskNotifyTake](https://www.freertos.org/ulTaskNotifyTake.html)
- [x] [xTaskNotify](https://www.freertos.org/xTaskNotify.html)
- [x] [xTaskNotifyAndQuery](https://www.freertos.org/xTaskNotifyAndQuery.html)
- [x] [xTaskNotifyAndQueryFromISR](https://www.freertos.org/xTaskNotifyAndQueryFromISR.html)
- [x] [xTaskNotifyFromISR](https://www.freertos.org/xTaskNotifyFromISR.html)
- [x] [xTaskNotifyWait](https://www.freertos.org/xTaskNotifyWait.html)
- [x] [xTaskNotifyStateClear](https://www.freertos.org/xTaskNotifyStateClear.html)
- [x] [ulTaskNotifyValueClear](https://www.freertos.org/ulTasknotifyValueClear.html)
### 2.4 消息队列
- [x] [xQueueCreate](https://www.freertos.org/a00116.html)
- [x] [xQueueCreateStatic](https://www.freertos.org/xQueueCreateStatic.html)
- [x] [vQueueDelete](https://www.freertos.org/a00018.html#vQueueDelete)
- [x] [xQueueSend](https://www.freertos.org/a00117.html)
- [x] [xQueueSendFromISR](https://www.freertos.org/a00119.html)
- [x] [xQueueSendToBack](https://www.freertos.org/xQueueSendToBack.html)
- [x] [xQueueSendToBackFromISR](https://www.freertos.org/xQueueSendToBackFromISR.html)
- [x] [xQueueSendToFront](https://www.freertos.org/xQueueSendToFront.html) (不支持设置超时)
- [x] [xQueueSendToFrontFromISR](https://www.freertos.org/xQueueSendToFrontFromISR.html)
- [x] [xQueueReceive](https://www.freertos.org/a00118.html)
- [x] [xQueueReceiveFromISR](https://www.freertos.org/a00120.html)
- [x] [uxQueueMessagesWaiting](https://www.freertos.org/a00018.html#ucQueueMessagesWaiting)
- [x] [uxQueueMessagesWaitingFromISR](https://www.freertos.org/a00018.html#ucQueueMessagesWaitingFromISR)
- [x] [uxQueueSpacesAvailable](https://www.freertos.org/a00018.html#uxQueueSpacesAvailable)
- [x] [xQueueReset](https://www.freertos.org/a00018.html#xQueueReset)
- [ ] [xQueueOverwrite](https://www.freertos.org/xQueueOverwrite.html)
- [ ] [xQueueOverwriteFromISR](https://www.freertos.org/xQueueOverwriteFromISR.html)
- [ ] [xQueuePeek](https://www.freertos.org/xQueuePeek.html)
- [ ] [xQueuePeekFromISR](https://www.freertos.org/xQueuePeekFromISR.html)
- [x] [xQueueIsQueueFullFromISR](https://www.freertos.org/a00018.html#xQueueIsQueueFullFromISR)
- [x] [xQueueIsQueueEmptyFromISR](https://www.freertos.org/a00018.html#xQueueIsQueueEmptyFromISR)
- [ ] [vQueueAddToRegistry](https://www.freertos.org/vQueueAddToRegistry.html)
- [ ] [vQueueUnregisterQueue](https://www.freertos.org/vQueueUnregisterQueue.html)
- [ ] [pcQueueGetName](https://www.freertos.org/pcQueueGetName.html)
### 2.5 信号量/互斥量
- [x] [xSemaphoreCreateBinary](https://www.freertos.org/xSemaphoreCreateBinary.html)
- [x] [xSemaphoreCreateBinaryStatic](https://www.freertos.org/xSemaphoreCreateBinaryStatic.html)
- [x] [vSemaphoreCreateBinary](https://www.freertos.org/a00121.html)
- [x] [xSemaphoreCreateCounting](https://www.freertos.org/CreateCounting.html)
- [x] [xSemaphoreCreateCountingStatic](https://www.freertos.org/xSemaphoreCreateCountingStatic.html)
- [x] [xSemaphoreCreateMutex](https://www.freertos.org/CreateMutex.html)
- [x] [xSemaphoreCreateMutexStatic](https://www.freertos.org/xSemaphoreCreateMutexStatic.html)
- [x] [xSem'CreateRecursiveMutex](https://www.freertos.org/xSemaphoreCreateRecursiveMutex.html)
- [x] [xSem'CreateRecursiveMutexStatic](https://www.freertos.org/xSemaphoreCreateRecursiveMutexStatic.html)
- [x] [vSemaphoreDelete](https://www.freertos.org/a00113.html#vSemaphoreDelete)
- [x] [xSemaphoreGetMutexHolder](https://www.freertos.org/xSemaphoreGetMutexHolder.html)
- [x] [uxSemaphoreGetCount](https://www.freertos.org/uxSemaphoreGetCount.html)
- [x] [xSemaphoreTake](https://www.freertos.org/a00122.html)
- [x] [xSemaphoreTakeFromISR](https://www.freertos.org/xSemaphoreTakeFromISR.html)
- [x] [xSemaphoreTakeRecursive](https://www.freertos.org/xSemaphoreTakeRecursive.html)
- [x] [xSemaphoreGive](https://www.freertos.org/a00123.html)
- [x] [xSemaphoreGiveRecursive](https://www.freertos.org/xSemaphoreGiveRecursive.html)
- [x] [xSemaphoreGiveFromISR](https://www.freertos.org/a00124.html)
### 2.6 定时器
- [x] [xTimerCreate](https://www.freertos.org/FreeRTOS-timers-xTimerCreate.html)
- [x] [xTimerCreateStatic](https://www.freertos.org/xTimerCreateStatic.html)
- [x] [xTimerIsTimerActive](https://www.freertos.org/FreeRTOS-timers-xTimerIsTimerActive.html)
- [x] [xTimerStart](https://www.freertos.org/FreeRTOS-timers-xTimerStart.html)
- [x] [xTimerStop](https://www.freertos.org/FreeRTOS-timers-xTimerStop.html)
- [x] [xTimerChangePeriod](https://www.freertos.org/FreeRTOS-timers-xTimerChangePeriod.html)
- [x] [xTimerDelete](https://www.freertos.org/FreeRTOS-timers-xTimerDelete.html)
- [x] [xTimerReset](https://www.freertos.org/FreeRTOS-timers-xTimerReset.html)
- [x] [xTimerStartFromISR](https://www.freertos.org/FreeRTOS-timers-xTimerStartFromISR.html)
- [x] [xTimerStopFromISR](https://www.freertos.org/FreeRTOS-timers-xTimerStopFromISR.html)
- [x] [xTimerChangePeriodFromISR](https://www.freertos.org/FreeRTOS-timers-xTimerChangePeriodFromISR.html)
- [x] [xTimerResetFromISR](https://www.freertos.org/FreeRTOS-timers-xTimerResetFromISR.html)
- [x] [pvTimerGetTimerID](https://www.freertos.org/FreeRTOS-timers-pvTimerGetTimerID.html)
- [x] [vTimerSetReloadMode](https://www.freertos.org/FreeRTOS-Timers-vTimerSetReloadMode.html)
- [x] [vTimerSetTimerID](https://www.freertos.org/FreeRTOS-timers-vTimerSetTimerID.html)
- [x] [xTimerGetTimerDaemonTaskHandle](https://www.freertos.org/FreeRTOS-Software-Timer-API-Functions.html#xTimerGetTimerDaemonTaskHandle)
- [ ] [xTimerPendFunctionCall](https://www.freertos.org/xTimerPendFunctionCall.html)
- [ ] [xTimerPendFunctionCallFromISR](https://www.freertos.org/xTimerPendFunctionCallFromISR.html)
- [x] [pcTimerGetName](https://www.freertos.org/FreeRTOS-timers-pcTimerGetName.html)
- [x] [xTimerGetPeriod](https://www.freertos.org/FreeRTOS-timers-xTimerGetPeriod.html)
- [x] [xTimerGetExpiryTime](https://www.freertos.org/FreeRTOS-timers-xTimerGetExpiryTime.html)
- [x] [uxTimerGetReloadMode](https://www.freertos.org/uxTimerGetReloadMode.html)
### 2.7 事件集
- [x] [xEventGroupCreate](https://www.freertos.org/xEventGroupCreate.html)
- [x] [xEventGroupCreateStatic](https://www.freertos.org/xEventGroupCreateStatic.html)
- [x] [vEventGroupDelete](https://www.freertos.org/vEventGroupDelete.html)
- [x] [xEventGroupWaitBits](https://www.freertos.org/xEventGroupWaitBits.html)
- [x] [xEventGroupSetBits](https://www.freertos.org/xEventGroupSetBits.html)
- [x] [xEventGroupSetBitsFromISR](https://www.freertos.org/xEventGroupSetBitsFromISR.html)
- [x] [xEventGroupClearBits](https://www.freertos.org/xEventGroupClearBits.html)
- [x] [xEventGroupClearBitsFromISR](https://www.freertos.org/xEventGroupClearBitsFromISR.html)
- [x] [xEventGroupGetBits](https://www.freertos.org/xEventGroupGetBits.html)
- [x] [xEventGroupGetBitsFromISR](https://www.freertos.org/xEventGroupGetBitsFromISR.html)
- [ ] [xEventGroupSync](https://www.freertos.org/xEventGroupSync.html)
### 2.8 不支持的功能
- [ ] [消息队列集](https://www.freertos.org/RTOS-queue-sets.html)
- [ ] [流缓冲区](https://www.freertos.org/RTOS-stream-buffer-API.html)
- [ ] [消息缓冲区](https://www.freertos.org/RTOS-message-buffer-API.html)
- [ ] [MPU](https://www.freertos.org/FreeRTOS-MPU-specific.html)
- [ ] [协程](https://www.freertos.org/croutineapi.html)
- [ ] [钩子函数](https://www.freertos.org/a00016.html)
- [ ] [跟踪功能](https://www.freertos.org/rtos-trace-macros.html)

## 3 使用注意事项
一些函数在功能和使用方法上和FreeRTOS略有不同，在迁移过程中需要注意。

### 3.1线程、消息队列与互斥量

#### 3.1.1 vTaskSuspend
`vTaskSuspend`只支持挂起当前运行的线程，在使用时`xTaskToSuspend`参数必须为`NULL`。否则会触发断言。

#### 3.1.2 xQueueSendToFront
`xQueueSendToFront`不支持设置超时，使用时`xTicksToWait`参数会被忽略，消息队列没有空间时会立即返回`errQUEUE_FULL`。

#### 3.1.3 xQueueCreateStatic
静态消息队列需要参考以下的例子创建，确保为消息队列分配的内存足够大：
```c
#define QUEUE_LENGTH 10
#define ITEM_SIZE sizeof( uint32_t )

/* 以下是在原版FreeRTOS分配内存的方法，由于RT-Thread消息队列内部的实现与FreeRTOS不同，这样分配的内存不够存放QUEUE_LENGTH个消息 */
//uint8_t ucQueueStorage[ QUEUE_LENGTH * ITEM_SIZE ];
/* 要使用QUEUE_BUFFER_SIZE宏分配内存 */
uint8_t ucQueueStorage[ QUEUE_BUFFER_SIZE(QUEUE_LENGTH, ITEM_SIZE)];
StaticQueue_t xQueueBuffer;
QueueHandle_t xQueue1;
xQueue1 = xQueueCreate( QUEUE_LENGTH, ITEM_SIZE, &( ucQueueStorage[ 0 ] ), &xQueueBuffer );
```
#### 3.1.4 Mutex和Recursive Mutex
FreeRTOS提供了两种互斥量，Mutex和Recursive Mutex。Recursive Mutex可以由同一个线程重复获取，Mutex不可以。RT-Thread提供的互斥量是可以重复获取的，因此兼容层也不对Mutex和Recursive Mutex做区分。用`xSemaphoreCreateMutex`和`xSemaphoreCreateRecursiveMutex`创建的互斥量都是可以重复获取的。
### 3.2 定时器
和FreeRTOS不同，RT-Thread不使用一个消息队列向定时器线程传递命令。使用兼容层时任何需要设置超时的定时器函数，如`xTimerStart( xTimer, xTicksToWait )`，`xTicksToWait`参数会被忽略，函数会立即完成命令并返回。
### 3.3 FromISR函数
FreeRTOS为一些函数提供了在中断中使用的FromISR版本，如果这些函数唤醒了更高优先级的线程，需要手动调度，如下所示：
```c
BaseType_t xHigherPrioritTaskWoken = pdFALSE;
xQueueSendToFrontFromISR( xRxQueue, &cIn, &xHigherPriorityTaskWoken );
if( xHigherPriorityTaskWoken )
{
  taskYIELD ();
}
```
RT-Thread不为函数提供FromISR版本，函数可以在中断调用并在内部完成调度。因此在兼容层中使用FromISR函数后不需要手动调度，`xHigherPriorityTaskWoken`总会被设置成`pdFALSE`。
### 3.4 内存堆
兼容层保留了FreeRTOS的五种内存分配算法，默认使用`heap_3`，`pvPortMalloc/vPortFree`内部调用`RT_KERNEL_MALLOC/RT_KERNEL_FREE`在RT-Thread内部的内存堆分配。这种情况下内存堆的大小由RT-Thread BSP配置决定，无法在`FreeRTOSConfig.h`中通过`configTOTAL_HEAP_SIZE`设置。
若使用其他算法，需要修改`FreeRTOS/sSConscript`，选择相应的源文件

```c
#可将heap_3.c替换成heap_1.c等
src += Glob(os.path.join("portable", "MemMang", "heap_3.c"))
```
在`FreeRTOS/portable/rt-thread/FreeRTOSConfig.h`中通过`configTOTAL_HEAP_SIZE`设置内存堆大小。应用调用`pvPortMalloc/vPortFree`会在一块独立于RT-Thread，大小为`configTOTAL_HEAP_SIZE`的内存堆中分配，RT-Thread内部的内存堆仍然存在，兼容层函数内部分配内存都在RT-Thread的内存堆完成。

### 3.5 线程优先级
RT-Thread线程优先级数值越小时优先级越高，而FreeRTOS线程优先级数值越大优先级越高。在使用兼容层的FreeRTOS API，如`xTaskCreate`，使用FreeRTOS的规则为线程指定优先级即可。若在应用中将RT-Thread和FreeRTOS API混合使用，在指定线程优先级时要特别注意。可以使用以下两个宏对RT-Thread和FreeRTOS线程优先级做转换：
```c
#define FREERTOS_PRIORITY_TO_RTTHREAD(priority)    ( configMAX_PRIORITIES - 1 - ( priority ) )
#define RTTHREAD_PRIORITY_TO_FREERTOS(priority)    ( RT_THREAD_PRIORITY_MAX - 1 - ( priority ) )
```

### 3.6 线程堆栈大小
FreeRTOS线程堆栈大小的单位为`sizeof(StackType_t)`，RT-Thread线程堆栈大小为`sizeof(rt_uint8_t)`。使用FreeRTOS API创建线程时一定要遵守FreeRTOS的规则，切勿混淆。

### 3.7 vTaskStartScheduler
由于RT-Thread和FreeRTOS的内核启动流程不同，使用兼容层时，`main`函数是在一个线程中运行，该线程优先级为`CONFIG_RT_MAIN_THREAD_PRIORITY`。（此选项通过SCons配置，数值越小优先级越高。），此时调度器已经开启。一般的FreeRTOS应用采用以下的方式创建线程：

```c
xTaskCreate(pxTask1Code, ......);
xTaskCreate(pxTask2Code, ......);
......
vTaskStartScheduler();
```

使用兼容层时，任何使用`xTaskCreate`创建的线程若优先级比`CONFIG_RT_MAIN_THREAD_PRIORITY`更高，会立即开始执行。`vTaskStartScheduler`只是为了提供对应用的兼容，没有任何实际效果。在使用兼容层时，创建线程要特别注意，确保在调用`xTaskCreate`时，该线程所需的所有资源已经完成初始化，可以正常运行。

## 4 使用方法

通过Env工具或RT-Thread Studio将FreeRTOS兼容层加入到工程中：

```shell
RT-Thread online packages
    system packages --->
        [*] FreeRTOS Wrapper --->
            Version (latest)
```

以下选项会影响到FreeRTOS兼容层：

```c
RT_USING_TIMER_SOFT /* 使用FreeRTOS定时器时必须开启*/
RT_TIMER_THREAD_PRIO  /* 定时器线程优先级。与FreeRTOS相反，该选项数值越小优先级越高 */
RT_TIMER_THREAD_STACK_SIZE  /* 定时器线程栈大小，单位为sizeof(rt_uint8_t) */
RT_USING_MUTEX  /* 使用FreeRTOS互斥量时必须开启*/
RT_USING_SEMAPHORE  /* 使用FreeRTOS信号量时必须开启*/
RT_USING_HEAP /* 使用FreeRTOS动态内存分配时必须开启*/
RT_TICK_PER_SECOND  /* 相当于FreeRTOS configTICK_RATE_HZ */
RT_THREAD_PRIORITY_MAX /* 相当于FreeRTOS configMAX_PRIORITIES */
RT_NAME_MAX /* 相当于FreeRTOS configMAX_TASK_NAME_LEN */
```

在`FreeRTOS/portable/rt-thread`提供了`FreeRTOSConfig.h`模版。大部分内容不可以修改或依赖RT-Thread内核的配置，可以手动修改的内容如下：

```c
/* 可以选择不使用recursive mutex */
#ifdef RT_USING_MUTEX
    #define configUSE_RECURSIVE_MUTEXES         1
    #define configUSE_MUTEXES                   1
#endif

/* 可以选择不使用counting semaphore */
#ifdef RT_USING_SEMAPHORE
    #define configUSE_COUNTING_SEMAPHORES       1
#endif

/* 若不使用heap_3，可以通过configTOTAL_HEAP_SIZE配置内存堆大小 */
#define configSUPPORT_STATIC_ALLOCATION         1
#ifdef RT_USING_HEAP
    #define configSUPPORT_DYNAMIC_ALLOCATION    1
    #define configTOTAL_HEAP_SIZE               10240
    #define configAPPLICATION_ALLOCATED_HEAP    0
#endif

#define configMINIMAL_STACK_SIZE                128

/* 可以选择的函数和功能 */
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
```
在`test`目录下提供了一些例程，可以将它们加入BSP目录下的`applications`文件夹中。使用SCons编译并烧录后，可以连接串口，输入相应的msh命令，观察例程的执行结果：
```shell
msh />queue_dynamic
Task 1 receive data 0 from queue
Task 1 receive data 1 from queue
Task 1 receive data 2 from queue
Task 1 receive data 3 from queue
Task 1 receive data 4 from queue
Task 1 receive data 5 from queue
Task 1 receive data 6 from queue
Task 1 receive data 7 from queue
Task 1 receive data 8 from queue
Task 1 receive data 9 from queue
Task 1 receive data 10 from queue
```

## 5 注意事项 & 补充说明

1、该版本`FreeRTOS`由乐鑫公司的`esp-idf`库中迁移而来，部分函数需要根据芯片来实现，如出现编译问题可参考[bsp/ESP32_C3](https://github.com/RT-Thread/rt-thread/tree/master/bsp/ESP32_C3)和[RT-Thread-packages/esp-idf](https://github.com/RT-Thread-packages/esp-idf/tree/master/components/freertos/FreeRTOS-Kernel)。

2、在`menuconfig`页面中可以开启`PKG_FREERTOS_USING_CONFIG_H`宏定义，该宏定义允许用户自定义相关`freertos`配置，具体使用案例可以参考[bsp/ESP32_C3/idf_port/include/freertos/FreeRTOSConfig.h](https://github.com/RT-Thread/rt-thread/blob/master/bsp/ESP32_C3/idf_port/include/freertos/FreeRTOSConfig.h)，注意只有部分配置宏定义可以被覆盖，详见`FreeRTOS/include/freertos/FreeRTOS.h`。

## 6 参考资料
RT-Thread文档 [https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README)

FreeRTOS文档 [https://www.freertos.org/a00106.html](https://www.freertos.org/a00106.html)

## 7 维护

主页：https://github.com/RT-Thread-packages/FreeRTOS-Wrapper

维护：[唐照洲](https://github.com/tangzz98)
