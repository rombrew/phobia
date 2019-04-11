/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION			1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE			0
#define configCPU_CLOCK_HZ			clock_cpu_hz
#define configTICK_RATE_HZ			1000
#define configMAX_PRIORITIES			5
#define configMINIMAL_STACK_SIZE		128
#define configMAX_TASK_NAME_LEN			24
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			1
#define configUSE_TASK_NOTIFICATIONS		0
#define configUSE_MUTEXES			1
#define configUSE_RECURSIVE_MUTEXES		0
#define configUSE_COUNTING_SEMAPHORES		0
#define configQUEUE_REGISTRY_SIZE		0
#define configUSE_QUEUE_SETS			0
#define configUSE_TIME_SLICING			1

#define configSUPPORT_DYNAMIC_ALLOCATION	1
#define configTOTAL_HEAP_SIZE			(24 * 1024)
#define configAPPLICATION_ALLOCATED_HEAP	0

#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			0
#define configCHECK_FOR_STACK_OVERFLOW		1
#define configUSE_MALLOC_FAILED_HOOK		1
#define configUSE_ASSERT_HOOK			1

#define configGENERATE_RUN_TIME_STATS		1
#define configUSE_TRACE_FACILITY		1
#define configUSE_STATS_FORMATTING_FUNCTIONS	1

#define configUSE_TIMERS			0

#define INCLUDE_vTaskDelete			1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay			1
#define INCLUDE_xTaskGetHandle			1

#define configPRIO_BITS       			4        /* 15 priority levels */

#define configKERNEL_INTERRUPT_PRIORITY 		(15 << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 		(5  << (8 - configPRIO_BITS))

#if (configUSE_ASSERT_HOOK == 1)
#define configASSERT(x)		if ((x) == pdFALSE) vAssertCalled(__FILE__, __LINE__)
#endif

#define vPortSVCHandler		irq_SVCall
#define xPortPendSVHandler	irq_PendSV
#define xPortSysTickHandler	irq_SysTick

extern unsigned long clock_cpu_hz;
extern void vAssertCalled(const char *file, int line);

#endif /* FREERTOS_CONFIG_H */

