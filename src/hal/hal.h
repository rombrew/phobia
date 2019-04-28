#ifndef _H_HAL_
#define _H_HAL_

#include "adc.h"
#include "can.h"
#include "flash.h"
#include "gpio.h"
#include "ppm.h"
#include "pwm.h"
#include "usart.h"
#include "wd.h"

#define GPIO_BOOST_12V			XGPIO_DEF2('B', 2)
#define GPIO_FAN			XGPIO_DEF2('B', 12)
#define GPIO_LED			XGPIO_DEF2('C', 12)

#define GPIO_HALL_A			XGPIO_DEF2('C', 6)
#define GPIO_HALL_B			XGPIO_DEF2('C', 7)
#define GPIO_HALL_C			XGPIO_DEF2('C', 8)

#define GPIO_SPI_NSS			XGPIO_DEF4('A', 4, 4, 5)
#define GPIO_SPI_SCK			XGPIO_DEF4('A', 5, 5, 5)
#define GPIO_SPI_MISO			XGPIO_DEF4('A', 6, 6, 5)
#define GPIO_SPI_MOSI			XGPIO_DEF4('A', 7, 7, 5)

#define CLOCK_APB1_HZ			(clock_cpu_hz / 4UL)
#define CLOCK_APB2_HZ			(clock_cpu_hz / 2UL)

#define __section_ccmram		__attribute__ (( section(".ccmram") ))
#define __section_ramfunc		__attribute__ (( section(".ramfunc"), used ))
#define __section_noinit		__attribute__ (( section(".noinit") ))

#define LOG_SIGNATURE			0x57575757UL

enum {
	LEG_A				= 1,
	LEG_B				= 2,
	LEG_C				= 4
};

enum {
	HALL_DISABLED			= 0,
	HALL_SENSOR,
	HALL_DRIVE_QEP,
};

enum {
	PPM_DISABLED			= 0,
	PPM_PULSE_WIDTH,
	PPM_STEP_DIR,
	PPM_CONTROL_QEP,
	PPM_I2C_BUS,
	PPM_DATA_USART,
};

typedef struct {

	int		HSE_crystal_clock;
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

	int		HALL_mode;
	int		HALL_sensor_state;

	unsigned long	CAN_msg_ID;
	int		CAN_msg_len;
	unsigned char	CAN_msg_payload[8];

	int		PPM_mode;
	int		PPM_timebase;
	int		PPM_signal_caught;
}
HAL_t;

typedef struct {

	char		text[512];
	int		signature;
	int		n;
}
LOG_t;

extern unsigned long		clock_cpu_hz;
extern HAL_t			hal;
extern LOG_t			log;

void hal_startup();
void hal_delay_us(int us);

void hal_system_reset();
void hal_sleep();
void hal_fence();

void log_putc(int c);
int log_validate();

extern void app_MAIN();

#endif /* _H_HAL_ */

