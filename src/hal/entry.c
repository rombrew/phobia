#include "libc.h"

#include "hal.h"
#include "cmsis/stm32xx.h"

extern u32_t ld_stack;
extern u32_t ld_begin_vectors;
extern u32_t ld_end_text;
extern u32_t ld_begin_data;
extern u32_t ld_end_data;
extern u32_t ld_begin_bss;
extern u32_t ld_end_bss;
extern u32_t ld_begin_ccm;
extern u32_t ld_end_ccm;
extern u32_t ld_end_flash;

void irq_Reset();
void irq_NMI();
void irq_HardFault();
void irq_MemoryFault();
void irq_BusFault();
void irq_UsageFault();
void irq_Default();
void irq_SVCall();
void irq_PendSV();
void irq_SysTick();

void irq_EXTI0();
void irq_ADC();
void irq_CAN1_TX();
void irq_CAN1_RX0();
void irq_CAN1_RX1();
void irq_CAN1_SCE();
void irq_TIM1_UP_TIM10();
void irq_TIM4();
void irq_USART3();

const FW_info_t		fw = {

	(u32_t) &ld_begin_vectors,
	(u32_t) &ld_end_flash,

	_HW_REVISION, __DATE__
};

__attribute__ (( section(".vectors"), used )) void * vectors[] = {

	(void *) &ld_stack,

	irq_Reset,
	irq_NMI,
	irq_HardFault,
	irq_MemoryFault,
	irq_BusFault,
	irq_UsageFault,
	(void *) &fw,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_SVCall,
	irq_Default,
	irq_Default,
	irq_PendSV,
	irq_SysTick,

	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_EXTI0,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_ADC,
	irq_CAN1_TX,
	irq_CAN1_RX0,
	irq_CAN1_RX1,
	irq_CAN1_SCE,
	irq_Default,
	irq_Default,
	irq_TIM1_UP_TIM10,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_TIM4,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_USART3,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default,
	irq_Default
};

static void
init_data(const u32_t *long_s, u32_t *long_d, u32_t *long_e)
{
	while (long_d < long_e) { *long_d++ = *long_s++; }
}

static void
init_bss(u32_t *long_d, u32_t *long_e)
{
	while (long_d < long_e) { *long_d++ = 0; }
}

void irq_Reset()
{
	hal_bootload();

	init_data(&ld_end_text, &ld_begin_data, &ld_end_data);
	init_bss(&ld_begin_bss, &ld_end_bss);
	init_bss(&ld_begin_ccm, &ld_end_ccm);

	hal_startup();
	app_MAIN();

	hal_system_reset();
}

