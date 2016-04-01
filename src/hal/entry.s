
.syntax unified
.cpu cortex-m4
.thumb

.section .vectors

	.word	ldStack

	.word	irqReset
	.word	irqNMI
	.word	irqHardFault
	.word	irqMemoryFault
	.word	irqBusFault
	.word	irqUsageFault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqSVCall
	.word	irqDefault
	.word	irqDefault
	.word	irqPendSV
	.word	irqSysTick

	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDMA1_Stream3
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqADC
	.word	irqDefault
	.word	irqCAN1_RX0
	.word	irqCAN1_RX1
	.word	irqCAN1_SCE
	.word	irqDefault
	.word	irqDefault
	.word	irqTIM1_UP_TIM10
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqUSART3
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault

.section .text

.type irqDefault, function
.align

irqDefault:

	b.n	irqDefault


.global irqReset
.type irqReset, function
.align

irqReset:

	ldr	r2, =(ldSdata)
	ldr	r1, =(ldEtext)
	ldr	r3, =(ldEdata)

	b.n	__iRdataComp

__iRdataLoop:

	ldr.w	r0, [r1], #4
	str.w	r0, [r2], #4

__iRdataComp:

	cmp	r2, r3
	bne.n	__iRdataLoop

	ldr	r2, =(ldSbss)
	ldr	r1, =(ldEbss)
	mov	r0, #0

	b.n	__iRbssComp

__iRbssLoop:

	str.w	r0, [r2], #4

__iRbssComp:

	cmp	r2, r1
	bne.n	__iRbssLoop

	bl	halStart
	bl	halMain

	bx	lr


