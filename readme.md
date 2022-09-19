# RT-Thread操作系统的FreeRTOS兼容层
## FreeRTOS Application Compatibility Layer (ACL) for RT-Thread
## 让基于FreeRTOS开发的应用层无感地迁移到RT-Thread操作系统

# 1 概述
这是一个针对RT-Thread国产操作系统的FreeRTOS操作系统兼容层，可以让基于FreeRTOS操作系统的项目快速、无感地迁移到RT-Thread操作系统上。项目基于FreeRTOS V10.4.6版本。

# 2 FreeRTOS功能支持情况
兼容层对FreeRTOS的支持情况记录在[issue](https://github.com/RT-Thread-packages/FreeRTOS-Wrapper/discussions/31)中记录。一些支持的函数在功能和使用方法上和FreeRTOS略有不同，在迁移过程中需要注意。
### vTaskSuspend
`vTaskSuspend`只支持挂起当前运行的线程，在使用时`xTaskToSuspend`参数必须为`NULL`。否则会触发断言。
### xQueueSendToFront
`xQueueSendToFront`不支持设置超时，使用时`xTicksToWait`参数会被忽略，消息队列没有空间时会立即返回`errQUEUE_FULL`。
### xQueueCreateStatic
静态消息队列需要参考以下的例子创建，确保为消息队列分配的内存足够大：
```
#define QUEUE_LENGTH 10
#define ITEM_SIZE sizeof( uint32_t )

/* 以下是在原版FreeRTOS分配内存的方法，由于RT-Thread消息队列内部的实现与FreeRTOS不同，这样分配的内存不够存放ITEM_SIZE个消息 */
//uint8_t ucQueueStorage[ QUEUE_LENGTH * ITEM_SIZE ];
/* 要使用QUEUE_BUFFER_SIZE宏分配内存 */
uint8_t ucQueueStorage[ QUEUE_BUFFER_SIZE(QUEUE_LENGTH, ITEM_SIZE)];
StaticQueue_t xQueueBuffer;
QueueHandle_t xQueue1;
xQueue1 = xQueueCreate( QUEUE_LENGTH, ITEM_SIZE, &( ucQueueStorage[ 0 ] ), &xQueueBuffer );
```
### Mutex和Recursive Mutex
FreeRTOS提供了两种互斥量，Mutex和Recursive Mutex。Recursive Mutex可以由同一个线程重复获取，Mutex不可以。RT-Thread提供的互斥量是可以重复获取的，因此兼容层也不对Mutex和Recursive Mutex做区分。用`xSemaphoreCreateMutex`和`xSemaphoreCreateRecursiveMutex`创建的互斥量都是可以重复获取的。
### 定时器
和FreeRTOS不同，RT-Thread不使用一个消息队列向定时器线程传递命令。使用兼容层时任何需要设置超时的定时器函数，如`xTimerStart( xTimer, xTicksToWait )`，`xTicksToWait`参数会被忽略，函数会立即完成命令并返回。
### FromISR函数
FreeRTOS为一些函数提供了在中断中使用的FromISR版本，如果这些函数唤醒了更高优先级的线程，需要手动调度，如下所示：
```
BaseType_t xHigherPrioritTaskWoken = pdFALSE;
xQueueSendToFrontFromISR( xRxQueue, &cIn, &xHigherPriorityTaskWoken );
if( xHigherPriorityTaskWoken )
{
  taskYIELD ();
}
```
RT-Thread不为函数提供FromISR版本，函数可以在中断调用并在内部完成调度。因此在兼容层中使用FromISR函数后不需要手动调度，`xHigherPriorityTaskWoken`总会被设置成`pdFALSE`。
### 内存堆
兼容层保留了FreeRTOS的五种内存分配算法，默认使用`heap_3`，`pvPortMalloc/vPortFree`内部调用`RT_KERNEL_MALLOC/RT_KERNEL_FREE`在RT-Thread内部的内存堆分配。这种情况下内存堆的大小由RT-Thread BSP配置决定，无法在`FreeRTOSConfig.h`中通过`configTOTAL_HEAP_SIZE`设置。
若使用其他算法，需要修改`FreeRTOS/sSConscript`，选择相应的源文件
```
# 可将heap_3.c替换成heap_1.c等
src += Glob(os.path.join("portable", "MemMang", "heap_3.c"))
```
在`FreeRTOS/portable/rt-thread/FreeRTOSConfig.h`中通过`configTOTAL_HEAP_SIZE`设置内存堆大小。应用调用`pvPortMalloc/vPortFree`会在一块独立于RT-Thread，大小为`configTOTAL_HEAP_SIZE`的内存堆中分配，RT-Thread内部的内存堆仍然存在，兼容层函数内部分配内存都在RT-Thread的内存堆完成。
### 线程优先级
RT-Threa线程优先级数值越小时优先级越高，而FreeRTOS线程优先级数值越大优先级越高。在使用兼容层的FreeRTOS API，如`xTaskCreate`，使用FreeRTOS的规则为线程指定优先级即可。若在应用中将RT-Thread和FreeRTOS API混合使用，在指定线程优先级时要特别注意。可以使用以下两个宏对RT-Thread和FreeRTOS线程优先级做转换：
```
#define FREERTOS_PRIORITY_TO_RTTHREAD(priority)    ( configMAX_PRIORITIES - 1 - ( priority ) )
#define RTTHREAD_PRIORITY_TO_FREERTOS(priority)    ( RT_THREAD_PRIORITY_MAX - 1 - ( priority ) )
```
### 线程堆栈大小
FreeRTOS线程堆栈大小的单位为`sizeof(StackType_t)`，RT-Thread线程堆栈大小为`sizeof(rt_uint8_t)`。使用FreeRTOS API创建线程时一定要遵守FreeRTOS的规则，切勿混淆。
### vTaskStartScheduler
由于RT-Thread和FreeRTOS的内核启动流程不同，使用兼容层时，`main`函数是在一个线程中运行，该线程优先级为`CONFIG_RT_MAIN_THREAD_PRIORITY`。（此选项通过SCons配置，数值越小优先级越高。），此时调度器已经开启。一般的FreeRTOS应用采用以下的方式创建线程：
```
xTaskCreate(pxTask1Code, ......);
xTaskCreate(pxTask2Code, ......);
......
vTaskStartScheduler();
```
使用兼容层时，任何使用`xTaskCreate`创建的线程若优先级比`CONFIG_RT_MAIN_THREAD_PRIORITY`更高，会立即开始执行。`vTaskStartScheduler`只是为了提供对应用的兼容，没有任何实际效果。在使用兼容层时，创建线程要特别注意，确保在调用`xTaskCreate`时，该线程所需的所有资源已经完成初始化，可以正常运行。

# 3 使用方法
通过Env工具将兼容层加入到工程中：
```
RT-Thread online packages
    system packages --->
        [*] FreeRTOS Wrapper --->
            Version (latest)
```
使用`scons --menuconfig`配置RT-Thread内核，以下选项会影响到FreeRTOS兼容层：
```
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
```
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
```
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

# 4 参考资料
RT-Thread文档 [https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README)
FreeRTOS文档 [https://www.freertos.org/a00106.html](https://www.freertos.org/a00106.html)