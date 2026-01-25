#ifndef _H_HAL_
#define _H_HAL_

#include "libc.h"
#include "hwdefs.h"

#include "adc.h"
#ifdef HW_HAVE_NETWORK_EPCAN
#include "can.h"
#endif /* HW_HAVE_NETWORK_EPCAN */
#include "dac.h"
#include "dps.h"
#ifdef HW_HAVE_DRV_ON_PCB
#include "drv.h"
#endif /* HW_HAVE_DRV_ON_PCB */
#include "flash.h"
#include "gpio.h"
#include "ppm.h"
#include "pwm.h"
#include "rng.h"
#include "spi.h"
#ifdef HW_HAVE_STEP_DIR_KNOB
#include "step.h"
#endif /* HW_HAVE_STEP_DIR_KNOB */
#include "tim.h"
#include "usart.h"
#ifdef HW_HAVE_USB_CDC_ACM
#include "usb.h"
#endif /* HW_HAVE_USB_CDC_ACM */
#include "wd.h"

#define LD_IRQ				__attribute__ ((naked))
#define LD_RAMFUNC			__attribute__ ((section(".ramfunc")))	\
					__attribute__ ((noinline, used))

#define LD_CCRAM			__attribute__ ((section(".ccram")))
#define LD_NOINIT			__attribute__ ((section(".noinit")))
#define LD_DMA				__attribute__ ((aligned(32)))

#define CLOCK_APB1_HZ			(clock_cpu_hz / 4U)
#define CLOCK_APB2_HZ			(clock_cpu_hz / 2U)

#define CLOCK_TIM1_HZ			(CLOCK_APB2_HZ * 2U)
#define CLOCK_TIM8_HZ			(CLOCK_APB2_HZ * 2U)
#define CLOCK_TIM4_HZ			(CLOCK_APB1_HZ * 2U)
#define CLOCK_TIM7_HZ			(CLOCK_APB1_HZ * 2U)

#define GPIO_ADC_TEMPINT		XGPIO_DEF3('H', 0, 16)
#define GPIO_ADC_VREFINT		XGPIO_DEF3('H', 0, 17)

#define GPIO_SWDIO			XGPIO_DEF2('A', 13)
#define GPIO_SWCLK			XGPIO_DEF2('A', 14)

#define ADC_SAMPLE_ADVANCE		110

enum {
	MCU_ID_UNKNOWN			= 0,
	MCU_ID_STM32F405,
	MCU_ID_STM32F722,
	MCU_ID_GD32F405
};

enum {
	LEG_A				= 1U,
	LEG_B				= 2U,
	LEG_C				= 4U
};

enum {
	PARITY_NONE			= 0,
	PARITY_EVEN,
	PARITY_ODD
};

enum {
	DPS_DISABLED			= 0,
	DPS_DRIVE_HALL,
	DPS_DRIVE_EABI,
	DPS_DRIVE_ON_SPI
};

enum {
	PPM_DISABLED			= 0,
	PPM_PULSE_WIDTH,
	PPM_PULSE_OUTPUT
};

#ifdef HW_HAVE_STEP_DIR_KNOB
enum {
	STEP_DISABLED			= 0,
	STEP_ON_STEP_DIR,
	STEP_ON_CW_CCW
};
#endif /* HW_HAVE_STEP_DIR_KNOB */

enum {
	HAL_DISABLED			= 0,
	HAL_ENABLED
};

enum {
	HAL_OK				= 0,
	HAL_FAULT
};

typedef struct {

	uint32_t	ld_begin;
	uint32_t	ld_crc32;

	const char	hardware[64];
	const char	revision[64];
	const char	build[16];
}
fw_info_t;

typedef struct {

	int		MCU_ID;

	int		USART_baudrate;
	int		USART_parity;

	float		PWM_frequency;
	int		PWM_resolution;
	float		PWM_deadtime;
#ifdef HW_HAVE_PWM_STOP
	int		PWM_stop;
#endif /* HW_HAVE_PWM_STOP */

	float		ADC_reference_voltage;
	float		ADC_shunt_resistance;
	float		ADC_amplifier_gain;
	float		ADC_voltage_ratio;
	float		ADC_terminal_ratio;

	int		ADC_sample_time;
	int		ADC_sample_advance;

#ifdef HW_HAVE_ANALOG_KNOB
	float		ADC_knob_ratio;
#endif /* HW_HAVE_ANALOG_KNOB */

	float		ADC_current_A;
	float		ADC_current_B;
	float		ADC_current_C;
	float		ADC_voltage_U;
	float		ADC_voltage_A;
	float		ADC_voltage_B;
	float		ADC_voltage_C;

#if (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TSC)
	float		ADC_analog_SIN;
	float		ADC_analog_COS;
#endif /* ADC_SEQUENCE__ABC_UTT_TSC */

#ifdef HW_HAVE_NETWORK_EPCAN
	int		CAN_bitfreq;
	int		CAN_errate;

	CAN_msg_t	CAN_msg;
#endif /* HW_HAVE_NETWORK_EPCAN */

	int		DPS_mode;

	int		PPM_mode;
	int		PPM_frequency;

	int		STEP_mode;
	int		STEP_frequency;

#ifdef HW_HAVE_DRV_ON_PCB
	DRV_config_t	DRV;
#endif /* HW_HAVE_DRV_ON_PCB */

#ifdef HW_HAVE_ALT_FUNCTION
	int		ALT_current;
	int		ALT_voltage;
#endif /* HW_HAVE_ALT_FUNCTION */

	uint32_t	CNT_raw[4];
	float		CNT_diag[3];

	struct {

		float		GA;
		float		GU;
		float		GT;
		float		GS;
		float		TS[2];
#ifdef HW_HAVE_ANALOG_KNOB
		float		GK;
#endif /* HW_HAVE_ANALOG_KNOB */
	}
	const_ADC;

	float		const_CNT[2];
}
HAL_t;

typedef struct {

	uint32_t	boot_FLAG;
	uint32_t	boot_COUNT;

	char		text[2000];
	int		text_wp;
	int		text_rp;
}
LOG_t;

extern const fw_info_t		fw;

extern uint32_t			ld_text_begin;
extern uint32_t			clock_cpu_hz;

extern HAL_t			hal;
extern LOG_t			log;

void hal_bootload();
void hal_startup();

int hal_lock_irq();
void hal_unlock_irq(int irq);

void hal_system_reset();
void hal_bootload_reset();

void hal_cpu_sleep();
void hal_memory_fence();

int log_status();
void log_bootup();
void log_putc(int c);
void log_flush();
void log_clean();

void DBGMCU_mode_stop();

extern void log_TRACE(const char *fmt, ...);
extern void app_MAIN();

#endif /* _H_HAL_ */

