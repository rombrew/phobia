
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
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqDefault
	.word	irqADC
	.word	irqCAN1_TX
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

.global irqReset
.type irqReset, function
.align

irqReset:

	ldr	r2, =(0x40023800)
	ldr	r3, [r2, #48]
	orr.w	r3, r3, #1048576
	str	r3, [r2, #48]

	ldr	r2, =(ldSdata)
	ldr	r1, =(ldEtext)
	ldr	r3, =(ldEdata)

	b.n	dataComp

dataLoop:

	ldr.w	r0, [r1], #4
	str.w	r0, [r2], #4

dataComp:

	cmp	r2, r3
	bne.n	dataLoop

	ldr	r2, =(ldSbss)
	ldr	r1, =(ldEbss)
	mov	r0, #0

	b.n	bssComp

bssLoop:

	str.w	r0, [r2], #4

bssComp:

	cmp	r2, r1
	bne.n	bssLoop

	ldr	r2, =(ldSccm)
	ldr	r1, =(ldEccm)

	b.n	ccmComp

ccmLoop:

	str.w	r0, [r2], #4

ccmComp:

	cmp	r2, r1
	bne.n	ccmLoop

	bl	halStart
	bl	halMain

	bx	lr


