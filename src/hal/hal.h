#ifndef _H_HAL_
#define _H_HAL_

#include "libc.h"
#include "hwdefs.h"

#include "adc.h"
#ifdef HW_HAVE_NETWORK_EPCAN
#include "can.h"
#endif /* HW_HAVE_NETWORK_EPCAN */
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
#include "tim.h"
#include "usart.h"
#ifdef HW_HAVE_USB_CDC_ACM
#include "usb.h"
#endif /* HW_HAVE_USB_CDC_ACM */
#include "wd.h"

#define LD_CCRAM			__attribute__ ((section(".ccram")))
#define LD_RAMFUNC			__attribute__ ((section(".ramfunc"), noinline, used))
#define LD_NOINIT			__attribute__ ((section(".noinit")))

#define CLOCK_APB1_HZ			(clock_cpu_hz / 4U)
#define CLOCK_APB2_HZ			(clock_cpu_hz / 2U)

#define GPIO_ADC_TEMPINT		XGPIO_DEF3('H', 0, 16)
#define GPIO_ADC_VREFINT		XGPIO_DEF3('H', 0, 17)

#define GPIO_SWDIO			XGPIO_DEF2('A', 13)
#define GPIO_SWCLK			XGPIO_DEF2('A', 14)

#define GPIO_DAC_1			GPIO_SPI_NSS
#define GPIO_DAC_2			GPIO_SPI_SCK

enum {
	LEG_A				= 1U,
	LEG_B				= 2U,
	LEG_C				= 4U
};

enum {
	DPS_DISABLED			= 0,
	DPS_DRIVE_HALL,
	DPS_DRIVE_EABI,
	DPS_DRIVE_SOFTWARE,
};

enum {
	PPM_DISABLED			= 0,
	PPM_PULSE_WIDTH,
	PPM_OUTPULSE,
	PPM_STEP_DIR,
	PPM_BACKUP_EABI,
};

enum {
	OPT_GPIO_1_ON		= 1U,
	OPT_GPIO_2_ON		= 2U,
};

typedef struct {

	uint32_t	ld_begin;
	uint32_t	ld_end;

	const char	hwrevision[16];
	const char	build[16];
}
FW_info_t;

typedef struct {

	int		USART_baud_rate;

	float		PWM_frequency;
	int		PWM_resolution;
	float		PWM_deadtime;

	float		ADC_reference_voltage;
	float		ADC_shunt_resistance;
	float		ADC_amplifier_gain;
	float		ADC_voltage_ratio;
	float		ADC_terminal_ratio;
	float		ADC_terminal_bias;

	int		ADC_sampling_time;
	int		ADC_sampling_advance;

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

	int		DPS_mode;

#ifdef HW_HAVE_NETWORK_EPCAN
	CAN_msg_t	CAN_msg;
#endif /* HW_HAVE_NETWORK_EPCAN */

	int		PPM_mode;
	int		PPM_timebase;
	int		PPM_caught;

#ifdef HW_HAVE_DRV_ON_PCB
	DRV_config_t	DRV;
#endif /* HW_HAVE_DRV_ON_PCB */

	struct {

		float		GA;
		float		GU;
		float		GT[2];
		float		TEMP[2];
		float		GS;
#ifdef HW_HAVE_ANALOG_KNOB
		float		GK;
#endif /* HW_HAVE_ANALOG_KNOB */
	}
	const_ADC;
}
HAL_t;

typedef struct {

	uint32_t	boot_SIGNATURE;
	uint32_t	boot_COUNT;

	char		textbuf[1024];
	int		len;
}
LOG_t;

extern const FW_info_t		fw;

extern uint32_t			ld_begin_text;
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

void DBGMCU_mode_stop();

extern void log_TRACE(const char *fmt, ...);
extern void app_MAIN();

#endif /* _H_HAL_ */

