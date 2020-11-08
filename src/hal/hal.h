#ifndef _H_HAL_
#define _H_HAL_

#include "adc.h"
#include "can.h"
#include "flash.h"
#include "gpio.h"
#include "ppm.h"
#include "pwm.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "wd.h"

#define GPIO_BOOST_12V			XGPIO_DEF2('B', 2)
#define GPIO_FAN			XGPIO_DEF2('B', 12)
#define GPIO_LED			XGPIO_DEF2('C', 12)

#ifdef _HW_REV2

#undef GPIO_LED
#define GPIO_LED			XGPIO_DEF2('B', 3)

#endif /* _HW_REV2 */

#ifdef _HW_KLEN

#undef GPIO_LED
#define GPIO_LED			XGPIO_DEF2('C', 4)

#endif /* _HW_KLEN */

#define GPIO_SWDIO			XGPIO_DEF4('A', 13)
#define GPIO_SWCLK			XGPIO_DEF4('A', 14)

#define GPIO_SPI_NSS			XGPIO_DEF4('A', 4, 4, 5)
#define GPIO_SPI_SCK			XGPIO_DEF4('A', 5, 5, 5)
#define GPIO_SPI_MISO			XGPIO_DEF4('A', 6, 6, 5)
#define GPIO_SPI_MOSI			XGPIO_DEF4('A', 7, 7, 5)

#define GPIO_DAC_1			GPIO_SPI_NSS
#define GPIO_DAC_2			GPIO_SPI_SCK

#define CLOCK_APB1_HZ			(clock_cpu_hz / 4UL)
#define CLOCK_APB2_HZ			(clock_cpu_hz / 2UL)

#define	LD_CCMRAM			__attribute__ ((section(".ccmram")))
#define LD_RAMFUNC			__attribute__ ((section(".ramfunc"), noinline, used))
#define LD_NOINIT			__attribute__ ((section(".noinit")))

#define INIT_SIGNATURE			0x55775577UL

enum {
	LEG_A				= 1,
	LEG_B				= 2,
	LEG_C				= 4
};

enum {
	TIM_DISABLED			= 0,
	TIM_DRIVE_HALL,
	TIM_DRIVE_ABI,
};

enum {
	PPM_DISABLED			= 0,
	PPM_PULSE_WIDTH,
	PPM_STEP_DIR,
	PPM_OUTPULSE,
	PPM_BACKUP_ABI,
};

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
	float		ADC_analog_ratio;

	float		ADC_current_A;
	float		ADC_current_B;
	float		ADC_voltage_U;
	float		ADC_voltage_A;
	float		ADC_voltage_B;
	float		ADC_voltage_C;

	struct {

		float		GA;
		float		GU;
		float		GT[2];
		float		GS;
		float		TEMP[2];
	}
	ADC_const;

	int		TIM_mode;

	int		CAN_mode_NART;
	CAN_msg_t	CAN_msg;

	int		PPM_mode;
	int		PPM_timebase;
	int		PPM_signal_caught;
}
HAL_t;

typedef struct {

	int		boot_SIGNATURE;
	int		boot_COUNT;

	char		textbuf[512];
	int		tail;
}
LOG_t;

extern long			ld_begin_vectors;
extern unsigned long		clock_cpu_hz;
extern HAL_t			hal;
extern LOG_t			log;

void hal_bootload();
void hal_startup();
void hal_delay_ns(int ns);

int hal_lock_irq();
void hal_unlock_irq(int irq);

void hal_system_reset();
void hal_bootload_jump();

void hal_sleep();
void hal_fence();

int log_bootup();
void log_putc(int c);

void hal_DBGMCU_mode_stop();

extern void log_TRACE(const char *fmt, ...);
extern void app_MAIN();

#endif /* _H_HAL_ */

