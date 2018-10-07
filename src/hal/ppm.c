/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define GPIO_TIM4_CH1			XGPIO_DEF4('B', 6, 0, 2)
#define GPIO_TIM4_CH2			XGPIO_DEF4('B', 7, 0, 2)

#define CLOCK_TIM4_HZ			(CLOCK_APB1_HZ * 2UL)

typedef struct {

	float			us_per_tik;
}
HAL_PPM_t;

static HAL_PPM_t		hal_PPM;

void irqTIM4()
{
	int		SR;

	SR = TIM4->SR;

	if (SR & TIM_SR_CC2IF) {

		TIM4->SR = ~TIM_SR_CC2IF;
		TIM4->DIER = TIM_DIER_CC4IE;
		hal.PPM_signal_caught = 1;
	}
	else if (SR & TIM_SR_CC4IF) {

		TIM4->SR = ~TIM_SR_CC4IF;
		TIM4->DIER = TIM_DIER_CC2IE;
		hal.PPM_signal_caught = 0;
	}
}

void PPM_mode_pulse_width()
{
	hal.PPM_timebase = (hal.PPM_timebase < 1000000UL) ? 1000000UL
		: (hal.PPM_timebase > CLOCK_TIM4_HZ) ? CLOCK_TIM4_HZ : hal.PPM_timebase;

	TIM4->CR1 = 0;
	TIM4->CR2 = 0;
	TIM4->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2;
	TIM4->DIER = TIM_DIER_CC2IE;
	TIM4->CCMR1 = TIM_CCMR1_CC2S_1 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_CC1S_0;
	TIM4->CCMR2 = TIM_CCMR2_OC4M_0;
	TIM4->CCER = TIM_CCER_CC2P | TIM_CCER_CC2E | TIM_CCER_CC1E;
	TIM4->CNT = 0;
	TIM4->PSC = CLOCK_TIM4_HZ / hal.PPM_timebase - 1UL;
	TIM4->ARR = 65535;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 65535;

	hal_PPM.us_per_tik = 1000000.f * (float) (TIM4->PSC + 1UL) / (float) CLOCK_TIM4_HZ;
	hal.PPM_timebase = CLOCK_TIM4_HZ / (TIM4->PSC + 1UL);
	hal.PPM_signal_caught = 0;
}

void PPM_startup()
{
	/* Enable TIM4 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	if (hal.PPM_mode == PPM_PULSE_WIDTH) {

		PPM_mode_pulse_width();
	}
	else if (hal.PPM_mode == PPM_STEP_DIR) {

		/* TODO */
	}

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(TIM4_IRQn, 3);
	NVIC_EnableIRQ(TIM4_IRQn);

	/* Start TIM4.
	 * */
	TIM4->CR1 |= TIM_CR1_CEN;

	/* Enable TIM4 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_TIM4_CH1);
	GPIO_set_mode_FUNCTION(GPIO_TIM4_CH2);
}

static void
PPM_halt()
{
	TIM4->CR1 = 0;

	NVIC_DisableIRQ(TIM4_IRQn);

	RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
}

void PPM_set_configuration()
{
	if (hal.PPM_mode != PPM_DISABLED) {

		PPM_halt();
		PPM_startup();
	}
	else {
		PPM_halt();
	}
}

float PPM_get_PERIOD()
{
	float		us;

	us = (float) TIM4->CCR1 * hal_PPM.us_per_tik;

	return us;
}

float PPM_get_PULSE()
{
	float		us;

	us = (float) TIM4->CCR2 * hal_PPM.us_per_tik;

	return us;
}

