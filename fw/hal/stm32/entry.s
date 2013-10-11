
.syntax unified
.cpu cortex-m4
.thumb

.section .vectors

	.word	ld_stack

	.word	irq_reset
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_svcall
	.word	irq_default
	.word	irq_default
	.word	irq_pendsv
	.word	irq_systick

	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default
	.word	irq_default

.section .text

.type irq_default, function
.align

irq_default:

	b.n	irq_default


.global irq_reset
.type irq_reset, function
.align

irq_reset:

	ldr	r2, =(ld_sdata)
	ldr	r1, =(ld_etext)
	ldr	r3, =(ld_edata)

	b.n	__vr_data_comp

__vr_data_loop:

	ldr.w	r0, [r1], #4
	str.w	r0, [r2], #4

__vr_data_comp:

	cmp	r2, r3
	bne.n	__vr_data_loop

	ldr	r2, =(ld_sbss)
	ldr	r1, =(ld_ebss)
	mov	r0, #0

	b.n	__vr_bss_comp

__vr_bss_loop:

	str.w	r0, [r2], #4

__vr_bss_comp:

	cmp	r2, r1
	bne.n	__vr_bss_loop

	b.w	main


