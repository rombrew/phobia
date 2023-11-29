#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

#ifndef HW_DRV_FREQUENCY
#define HW_DRV_FREQUENCY		4000000U	/* (Hz) */
#endif /* HW_DRV_FREQUENCY */

static int
DRV_read_reg(int addr)
{
	uint16_t	txbuf, rxbuf;
	int		fault, raddr, data;

	txbuf = 0x8000U | ((addr & 0xFU) << 11);

	SPI_transfer(HW_DRV_ID_ON_PCB, txbuf);
	rxbuf = SPI_transfer(HW_DRV_ID_ON_PCB, 0x8000U);

	fault = (rxbuf & 0x8000U) ? 1 : 0;
	raddr = (rxbuf & 0x7800U) >> 11;

	data = 0x800U;

	if (fault == 0 && raddr == addr) {

		data = rxbuf & 0x7FFU;
	}

	return data;
}

static int
DRV_write_reg(int addr, int data)
{
	uint16_t	txbuf, rxbuf;
	int		fault;

	txbuf = ((addr & 0xFU) << 11) | (data & 0x7FFU);

	SPI_transfer(HW_DRV_ID_ON_PCB, txbuf);
	rxbuf = SPI_transfer(HW_DRV_ID_ON_PCB, 0x8000U);

	fault = (rxbuf & 0x8000U) ? 1 : 0;

	return fault;
}

static void
DRV8301_configure()
{
	int			config;

	config = (hal.DRV.gate_current & 0x3U);

	if (hal.DRV.ocp_level < 32) {

		config |= 0x010U | ((hal.DRV.ocp_level & 0x1FU) << 5);
	}
	else {
		config |= 0x630U;
	}

	DRV_write_reg(2, config);
	DRV_write_reg(3, 0);

	if (DRV_read_reg(2) != config) {

		log_TRACE("DRV8301 unable to configure" EOL);
	}

	hal.DRV.status_raw = DRV_read_reg(0);

	if (hal.DRV.status_raw != 0) {

		log_TRACE("DRV8301 fault %4x" EOL, hal.DRV.status_raw);
	}
}

static void
DRV8301_startup()
{
	GPIO_set_mode_OUTPUT(hal.DRV.gpio_GATE_EN);
	GPIO_set_HIGH(hal.DRV.gpio_GATE_EN);

	GPIO_set_mode_INPUT(hal.DRV.gpio_FAULT);
	GPIO_set_mode_PULL_UP(hal.DRV.gpio_FAULT);

	vTaskDelay((TickType_t) 20);

	SPI_startup(HW_DRV_ID_ON_PCB, HW_DRV_FREQUENCY, SPI_LOW_FALLING);

	DRV8301_configure();

	hal.DRV.gate_ON = 1;
}

void DRV_startup()
{
	if (hal.DRV.part == DRV_PART_DRV8301) {

		DRV8301_startup();
	}
	else if (hal.DRV.part == DRV_PART_DRV8305) {

		/* TODO */
	}
}

void DRV_halt()
{
	if (hal.DRV.gate_ON != 0) {

		hal.DRV.gate_ON = 0;

		SPI_halt(HW_DRV_ID_ON_PCB);

		GPIO_set_LOW(hal.DRV.gpio_GATE_EN);
		vTaskDelay((TickType_t) 100);
	}
}

void DRV_configure()
{
	if (hal.DRV.part == DRV_PART_DRV8301) {

		if (hal.DRV.gate_ON != 0) {

			DRV8301_configure();
		}
	}
	else if (hal.DRV.part == DRV_PART_DRV8305) {

		/* TODO */
	}
}

void DRV_status()
{
	if (hal.DRV.part == DRV_PART_DRV8301) {

		if (hal.DRV.gate_ON != 0) {

			hal.DRV.status_raw = DRV_read_reg(0);
		}
	}
	else if (hal.DRV.part == DRV_PART_DRV8305) {

		/* TODO */
	}
}

int DRV_fault()
{
	int		fault = 0;

	if (hal.DRV.gate_ON != 0) {

		fault = (GPIO_get_STATE(hal.DRV.gpio_FAULT) != 0) ? 0 : 1;
	}

	return fault;
}

