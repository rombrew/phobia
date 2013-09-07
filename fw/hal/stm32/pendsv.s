
.syntax unified
.cpu cortex-m4
.thumb

.section .text

.global irq_pendsv
.type irq_pendsv, function
.align

irq_pendsv:

	bl rt_task_shedule

	bx	lr

