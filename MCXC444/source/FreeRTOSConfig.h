/*
 * =============================================================================
 * PetPal - FreeRTOS Configuration
 * =============================================================================
 * File:    FreeRTOSConfig.h
 * Desc:    FreeRTOS kernel configuration for FRDM-MCXC444 (ARM Cortex-M0+)
 *
 * RUBRIC REQUIREMENTS MET:
 *   - configUSE_PREEMPTION  = 1   (pre-emption enabled)
 *   - configUSE_TIME_SLICING = 1  (time-slicing enabled)
 *
 * NOTE: If your MCUXpresso project already generates a FreeRTOSConfig.h,
 *       merge these settings into it rather than replacing it entirely.
 * =============================================================================
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ========================= CORE SETTINGS ================================= */

/* REQUIRED: Pre-emption enabled */
#define configUSE_PREEMPTION                     1

/* REQUIRED: Time-slicing enabled (equal-priority tasks share CPU time) */
#define configUSE_TIME_SLICING                   1

/* CPU clock frequency (adjust to your board's actual core clock) */
#define configCPU_CLOCK_HZ                       (48000000UL)

/* FreeRTOS tick rate: 1000Hz = 1ms tick resolution */
#define configTICK_RATE_HZ                       ((TickType_t)1000)

/* Maximum number of priority levels */
#define configMAX_PRIORITIES                     (6)

/* Minimum task stack size (in words, not bytes) */
#define configMINIMAL_STACK_SIZE                 ((unsigned short)128)

/* Total heap available for FreeRTOS dynamic allocation */
#define configTOTAL_HEAP_SIZE                    ((size_t)(12 * 1024))

/* Maximum task name length */
#define configMAX_TASK_NAME_LEN                  (10)

/* Use 16-bit ticks on Cortex-M0+ to save RAM (set to 0 for 32-bit) */
#define configUSE_16_BIT_TICKS                   0

/* Allow tasks to yield to same-priority tasks */
#define configIDLE_SHOULD_YIELD                  1

/* ========================= FEATURE ENABLES =============================== */

/* We use mutexes for UART and status protection */
#define configUSE_MUTEXES                        1

/* We use binary semaphores (from ISR → task) */
#define configUSE_COUNTING_SEMAPHORES            0

/* We use software timers (ultrasonic trigger timer) */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                (configMAX_PRIORITIES - 2)
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             (configMINIMAL_STACK_SIZE * 2)

/* Queue support (for actuator command and sensor data queues) */
#define configUSE_QUEUE_SETS                     0

/* Enable task notifications (lightweight alternative to semaphores) */
#define configUSE_TASK_NOTIFICATIONS             1

/* Runtime stats and trace (useful for debugging, can disable for production) */
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

/* Stack overflow detection (ENABLE during development!) */
#define configCHECK_FOR_STACK_OVERFLOW           2

/* Use idle hook for low-power sleep (optional) */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0

/* Malloc failed hook (ENABLE during development!) */
#define configUSE_MALLOC_FAILED_HOOK             1

/* ========================= API INCLUDES ================================== */

#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xResumeFromISR                   1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1

/* ========================= CORTEX-M0+ SPECIFIC =========================== */

/*
 * ARM Cortex-M0+ does not have BASEPRI register.
 * FreeRTOS uses PRIMASK for critical sections on M0/M0+.
 * These settings are specific to M0+ ports.
 */

/* Lowest interrupt priority (all bits set) */
#define configKERNEL_INTERRUPT_PRIORITY          (255)

/* Priority from which FreeRTOS API calls can be made from ISR */
/* On M0+, this is less relevant since there's no BASEPRI */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (191)

/* Map FreeRTOS port handlers to CMSIS names */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* Assert macro for debugging */
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for (;;); }


#endif /* FREERTOS_CONFIG_H */
