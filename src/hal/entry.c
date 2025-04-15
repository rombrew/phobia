#include "hal.h"
#include "cmsis/stm32xx.h"

#define LD_VTABLE		__attribute__ ((section(".vtab"), used))
#define LD_IRQ_WEAK		__attribute__ ((weak, alias("irq_Weak")))

extern uint32_t ld_stack;
extern uint32_t ld_text_begin;
extern uint32_t ld_text_end;
extern uint32_t ld_ramfunc_load;
extern uint32_t ld_ramfunc_begin;
extern uint32_t ld_ramfunc_end;
extern uint32_t ld_data_load;
extern uint32_t ld_data_begin;
extern uint32_t ld_data_end;
extern uint32_t ld_bss_begin;
extern uint32_t ld_bss_end;
extern uint32_t ld_ccm_begin;
extern uint32_t ld_ccm_end;
extern uint32_t ld_crc32_stub;

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

void irq_Weak() { irq_Default(); };

void irq_EXTI0() LD_IRQ_WEAK;
void irq_ADC() LD_IRQ_WEAK;
void irq_CAN1_TX() LD_IRQ_WEAK;
void irq_CAN1_RX0() LD_IRQ_WEAK;
void irq_CAN1_RX1() LD_IRQ_WEAK;
void irq_CAN1_SCE() LD_IRQ_WEAK;
void irq_TIM1_UP_TIM10() LD_IRQ_WEAK;
void irq_TIM4() LD_IRQ_WEAK;
void irq_USART1() LD_IRQ_WEAK;
void irq_USART2() LD_IRQ_WEAK;
void irq_USART3() LD_IRQ_WEAK;
void irq_TIM7() LD_IRQ_WEAK;
void irq_OTG_FS() LD_IRQ_WEAK;

const fw_info_t		fw = {

	(uint32_t) &ld_text_begin,
	(uint32_t) &ld_crc32_stub,

#include "hgdef.h"

	_HW_REV, _HG_REV, __DATE__
};

LD_VTABLE void *vtab[] = {

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
	irq_USART1,
	irq_USART2,
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
	irq_TIM7,
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
	irq_OTG_FS,
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
init_data(const uint32_t *long_s, uint32_t *long_d, const uint32_t *long_e)
{
	while (long_d < long_e) { * (long_d++) = * (long_s++); }
}

static void
init_bss(uint32_t *long_d, const uint32_t *long_e)
{
	while (long_d < long_e) { * (long_d++) = 0U; }
}

void irq_Reset()
{
	hal_bootload();

	init_data(&ld_ramfunc_load, &ld_ramfunc_begin, &ld_ramfunc_end);
	init_data(&ld_data_load, &ld_data_begin, &ld_data_end);

	init_bss(&ld_bss_begin, &ld_bss_end);
	init_bss(&ld_ccm_begin, &ld_ccm_end);

	hal_startup();
	app_MAIN();

	hal_system_reset();
}

