#define HW_MCU_STM32F405

#define HW_HAVE_LOW_SIDE_SHUNT
#define HW_HAVE_PWM_REVERSED
#define HW_HAVE_NTC_ON_PCB

#define HW_CLOCK_CRYSTAL_HZ		25000000U

#define HW_PWM_FREQUENCY_HZ		30000.f
#define HW_PWM_DEADTIME_NS		90.f		/* IRFH7185 */

#define HW_PWM_MINIMAL_PULSE		0.2f
#define HW_PWM_CLEARANCE_ZONE		5.0f
#define HW_PWM_SKIP_ZONE		2.0f
#define HW_PWM_BOOTSTRAP_RETENTION	20.f		/* UCC27211A  */

#define HW_ADC_SAMPLING_SEQUENCE	ADC_SEQUENCE__ABC_UTT_TXX

#define HW_ADC_REFERENCE_VOLTAGE	3.3f
#define HW_ADC_SHUNT_RESISTANCE		0.005f
#define HW_ADC_AMPLIFIER_GAIN		- 5.f		/* INA213 */

#define HW_ADC_VOLTAGE_R1		47000.f
#define HW_ADC_VOLTAGE_R2		2200.f
#define HW_ADC_VOLTAGE_R3		1000000000000.f		/* have no bias */

#define HW_NTC_PCB_TYPE			NTC_GND
#define HW_NTC_PCB_BALANCE		10000.f
#define HW_NTC_PCB_NTC0			10000.f
#define HW_NTC_PCB_TA0			25.f
#define HW_NTC_PCB_BETTA		3435.f

#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('B', 0, 8)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_CURRENT_C		XGPIO_DEF3('B', 1, 9)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('C', 0, 10)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_NTC_PCB		XGPIO_DEF3('A', 3, 3)

#define GPIO_USART3_TX			XGPIO_DEF4('B', 10, 0, 7)
#define GPIO_USART3_RX			XGPIO_DEF4('B', 11, 0, 7)

#define GPIO_FAN_EN			XGPIO_DEF2('B', 12)
#define GPIO_LED_ALERT			XGPIO_DEF2('C', 4)

