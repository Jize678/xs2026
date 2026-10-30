/**
  ******************************************************************************
  * @file    FreeRTOSConfig.h
  * @brief   FreeRTOS V10.5.1 内核配置 —— STM32F103C8T6 / Cortex-M3 / Keil AC5
  *
  *  目标芯片 : STM32F103C8T6（64KB Flash / 20KB SRAM），SYSCLK = 72MHz
  *  工具链   : ARMCC 5.06（Keil MDK，工程已开 --c99）
  *  移植层   : Middlewares\FreeRTOS\portable\RVDS\ARM_CM3
  *  内存方案 : heap_4.c（首次适配 + 相邻空闲块合并），堆大小 6KB
  *
  *  设计约定（与本工程 App\app_config.h 保持一致）：
  *    - 内核 tick = 1kHz（1ms），SysTick 由 FreeRTOS 接管；
  *    - 所有会调用 FreeRTOS API 的中断，其 NVIC 优先级必须 >= 5
  *      （即数值 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）；
  *    - 中断优先级分组保持 NVIC_PriorityGroup_4（4 位全给抢占优先级）；
  *    - 不使用低功耗 tickless，不使用静态内存分配。
  ******************************************************************************
  */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * 调度器基础配置
 *----------------------------------------------------------*/
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 72000000 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 6 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )
#define configMAX_TASK_NAME_LEN                 ( 8 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 0
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1

/*-----------------------------------------------------------
 * 同步原语
 *----------------------------------------------------------*/
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_QUEUE_SETS                    0
#define configQUEUE_REGISTRY_SIZE               0

/*-----------------------------------------------------------
 * 内存
 *----------------------------------------------------------*/
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 7 * 1024 ) )
#define configAPPLICATION_ALLOCATED_HEAP        0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2

/*-----------------------------------------------------------
 * 钩子函数
 *----------------------------------------------------------*/
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/*-----------------------------------------------------------
 * 软件定时器（LED 状态机、看门狗喂狗等使用）
 *----------------------------------------------------------*/
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 3 )
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            ( 128 )

/*-----------------------------------------------------------
 * 调试与统计
 *----------------------------------------------------------*/
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0

/*-----------------------------------------------------------
 * 中断优先级配置（Cortex-M3，4 位优先级）
 *
 * NVIC 优先级寄存器的高 4 位有效；FreeRTOS 用 BASEPRI 屏蔽
 * “优先级数值 >= configMAX_SYSCALL_INTERRUPT_PRIORITY”的中断，
 * 因此任何调用 xxxFromISR() 的中断都必须落在这个区间内。
 *----------------------------------------------------------*/
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
        ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/*-----------------------------------------------------------
 * 断言
 *----------------------------------------------------------*/
#define configASSERT( x ) \
        if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/*-----------------------------------------------------------
 * 内核异常/中断向量映射
 *   工程启动文件（startup_stm32f10x_md.s）里定义的是 SVC_Handler /
 *   PendSV_Handler / SysTick_Handler，这里把内核的三个处理函数映射过去，
 *   因此 stm32f10x_it.c 里不再实现这三个函数。
 *----------------------------------------------------------*/
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

/*-----------------------------------------------------------
 * 需要编译进来的 API
 *----------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

#endif /* FREERTOS_CONFIG_H */
