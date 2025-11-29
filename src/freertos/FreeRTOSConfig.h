/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
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

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configCPU_CLOCK_HZ				clock_cpu_hz

#define configTICK_RATE_HZ				1000
#define configUSE_PREEMPTION				1
#define configUSE_TIME_SLICING				1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION		1
#define configUSE_TICKLESS_IDLE				0
#define configMAX_PRIORITIES				5
#define configMINIMAL_STACK_SIZE			120
#define configMAX_TASK_NAME_LEN				16
#define configTICK_TYPE_WIDTH_IN_BITS			TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD				1
#define configQUEUE_REGISTRY_SIZE			0
#define configENABLE_BACKWARD_COMPATIBILITY		0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS		0

#define configUSE_NEWLIB_REENTRANT			0
#define configUSE_TIMERS				0
#define configUSE_TASK_NOTIFICATIONS			0
#define configUSE_MUTEXES				1
#define configUSE_RECURSIVE_MUTEXES			0
#define configUSE_COUNTING_SEMAPHORES			0
#define configUSE_QUEUE_SETS				0
#define configUSE_APPLICATION_TASK_TAG			0
#define configUSE_CO_ROUTINES				0

#define configSUPPORT_STATIC_ALLOCATION			0
#define configSUPPORT_DYNAMIC_ALLOCATION		1
#define configAPPLICATION_ALLOCATED_HEAP		1
#define configTOTAL_HEAP_SIZE				20000
#define configENABLE_HEAP_PROTECTOR			0

#define configUSE_IDLE_HOOK				1
#define configUSE_TICK_HOOK				0
#define configUSE_MALLOC_FAILED_HOOK			1
#define configCHECK_FOR_STACK_OVERFLOW			1

#define configGENERATE_RUN_TIME_STATS			0
#define configUSE_TRACE_FACILITY			1

#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay				1
#define INCLUDE_xTaskGetHandle				1

#define configDEFAULT_STACK_SIZE			190
#define configHUGE_STACK_SIZE				240

#define configPRIO_BITS			4        /* 15 priority levels */

#define configKERNEL_INTERRUPT_PRIORITY 	(15 << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	(5  << (8 - configPRIO_BITS))

/*
#define configASSERT(x)		if ((x) == pdFALSE) vAssertHook(__FILE__, __LINE__)
*/

#define configCHECK_HANDLER_INSTALLATION		0

#define vPortSVCHandler		irq_SVCall
#define xPortPendSVHandler	irq_PendSV
#define xPortSysTickHandler	irq_SysTick

extern uint32_t clock_cpu_hz;
extern void vAssertHook(const char *file, int line);

#endif /* FREERTOS_CONFIG_H */

