#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

#ifndef HW_DRV_FREQUENCY
#define HW_DRV_FREQUENCY		2000000U	/* (Hz) */
#endif /* HW_DRV_FREQUENCY */

#ifndef HW_DRV_FAULT_THRESHOLD
#define HW_DRV_FAULT_THRESHOLD		20
#endif /* HW_DRV_FAULT_THRESHOLD */

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
	int			config, check;

	config = (hal.DRV.gate_current & 0x3U);

	if (hal.DRV.ocp_level < 32) {

		config |= 0x010U | ((hal.DRV.ocp_level & 0x1FU) << 6);
	}
	else {
		config |= 0x630U;
	}

	DRV_write_reg(2, config);
	DRV_write_reg(3, 0);

	check = DRV_read_reg(2);

	if (check != config) {

		log_TRACE("DRV configuration fault %4x" EOL, check);
	}

	hal.DRV.status_raw = DRV_read_reg(0);

	if (hal.DRV.status_raw != 0) {

		log_TRACE("DRV status %4x" EOL, hal.DRV.status_raw);
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
}

static void
DRV8301_halt()
{
	SPI_halt(HW_DRV_ID_ON_PCB);

	GPIO_set_LOW(hal.DRV.gpio_GATE_EN);
	vTaskDelay((TickType_t) 100);
}

void DRV_startup()
{
	if (hal.DRV.partno == DRV_PART_DRV8301) {

		DRV8301_startup();
	}
	else if (hal.DRV.partno == DRV_PART_DRV8305) {

		/* TODO */
	}

	hal.DRV.partno_ENABLED = hal.DRV.partno;
}

void DRV_halt()
{
	if (hal.DRV.partno_ENABLED == DRV_PART_DRV8301) {

		DRV8301_halt();
	}
	else if (hal.DRV.partno_ENABLED == DRV_PART_DRV8305) {

		/* TODO */
	}

	hal.DRV.partno_ENABLED = DRV_NONE;
	hal.DRV.fault_CNT = 0;
}

void DRV_configure()
{
	if (hal.DRV.partno_ENABLED == DRV_PART_DRV8301) {

		DRV8301_configure();
	}
	else if (hal.DRV.partno_ENABLED == DRV_PART_DRV8305) {

		/* TODO */
	}
}

void DRV_status()
{
	if (hal.DRV.partno_ENABLED == DRV_PART_DRV8301) {

		hal.DRV.status_raw = DRV_read_reg(0);
	}
	else if (hal.DRV.partno_ENABLED == DRV_PART_DRV8305) {

		/* TODO */
	}
}

int DRV_fault()
{
	if (hal.DRV.partno_ENABLED != DRV_NONE) {

		if (unlikely(hal.DRV.fault_CNT >= HW_DRV_FAULT_THRESHOLD)) {

			return HAL_FAULT;
		}
		else {
			if (likely(GPIO_get_STATE(hal.DRV.gpio_FAULT) != 0)) {

				hal.DRV.fault_CNT = 0;
			}
			else {
				hal.DRV.fault_CNT += 1;
			}
		}
	}

	return HAL_OK;
}

float DRV_gate_current()
{
	float		current = 0.f;

	if (hal.DRV.partno_ENABLED == DRV_PART_DRV8301) {

		switch (hal.DRV.gate_current) {

			case 0:
				current = 1.7f;
				break;

			case 1:
				current = 0.7f;
				break;

			case 2:
				current = 0.25f;
				break;

			default:
				current = 0.f;
				break;
		}
	}
	else if (hal.DRV.partno_ENABLED == DRV_PART_DRV8305) {

		/* TODO */
	}

	return current;
}

float DRV_ocp_level()
{
	extern float	m_expf(float x);

	float		level = 0.f;

	if (hal.DRV.partno_ENABLED == DRV_PART_DRV8301) {

		if (hal.DRV.ocp_level < 32) {

			level = 0.06f * m_expf(hal.DRV.ocp_level * 0.119f);
		}
	}
	else if (hal.DRV.partno_ENABLED == DRV_PART_DRV8305) {

		/* TODO */
	}

	return level;
}

