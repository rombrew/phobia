#include "cmsis/stm32f4xx.h"
#include "hal/hal.h"

extern long ld_stack;
extern long ld_end_text;
extern long ld_begin_data;
extern long ld_end_data;
extern long ld_begin_bss;
extern long ld_end_bss;
extern long ld_begin_ccm;
extern long ld_end_ccm;

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

__attribute__ (( section(".vectors"), used )) void * vectors[] = {

	(void *) &ld_stack,

	irq_Reset,
	irq_NMI,
	irq_HardFault,
	irq_MemoryFault,
	irq_BusFault,
	irq_UsageFault,
	irq_Default,
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
__init_data(const long *s, long *d, long *e)
{
	while (d < e) { *d++ = *s++; }
}

static void
__init_bss(long *d, long *e)
{
	while (d < e) { *d++ = 0; }
}

void irq_Reset()
{
	__init_data(&ld_end_text, &ld_begin_data, &ld_end_data);
	__init_bss(&ld_begin_bss, &ld_end_bss);
	__init_bss(&ld_begin_ccm, &ld_end_ccm);

	hal_startup();
	app_MAIN();
	hal_system_reset();
}

