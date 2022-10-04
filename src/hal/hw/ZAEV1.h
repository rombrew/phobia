#define HW_MCU_STM32F405

#define HW_HAVE_NO_STEP_DIR
#define HW_HAVE_NTC_ON_PCB
#define HW_HAVE_NTC_MOTOR

#define HW_CLOCK_CRYSTAL_HZ		8000000UL

#define HW_PWM_FREQUENCY_HZ		30000.f
#define HW_PWM_DEADTIME_NS		420.f		/* IRF7749 */

#define HW_PWM_MINIMAL_PULSE		0.2f
#define HW_PWM_CLEARANCE_ZONE		5.0f
#define HW_PWM_SKIP_ZONE		2.0f
#define HW_PWM_BOOTSTRAP_RETENTION	5.f		/* IRS21867S + 0.1uF */

#define HW_ADC_SAMPLING_SCHEME		ADC_SEQUENCE__ABC_UTT_TXX

#define HW_ADC_REFERENCE_VOLTAGE	3.3f
#define HW_ADC_SHUNT_RESISTANCE		0.0001667f
#define HW_ADC_AMPLIFIER_GAIN		20.f		/* AD8418 */

#define HW_ADC_VOLTAGE_R1		39000.f
#define HW_ADC_VOLTAGE_R2		2200.f
#define HW_ADC_VOLTAGE_BIAS_R3		1000000000000.f		/* have no bias */

#define HW_NTC_PCB_TYPE			NTC_VCC
#define HW_NTC_PCB_BALANCE		10000.f
#define HW_NTC_PCB_NTC_0		10000.f
#define HW_NTC_PCB_TA_0			25.f
#define HW_NTC_PCB_BETTA		3380.f			/* unknown part */

#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('C', 0, 10)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('C', 1, 11)
#define GPIO_ADC_CURRENT_C		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_NTC_PCB		XGPIO_DEF3('A', 3, 3)
#define GPIO_ADC_NTC_EXT		XGPIO_DEF3('C', 4, 14)

#define GPIO_USART3_TX			XGPIO_DEF4('C', 10, 0, 7)
#define GPIO_USART3_RX			XGPIO_DEF4('C', 11, 0, 7)

#define GPIO_BOOST_EN			XGPIO_DEF2('A', 4)
#define GPIO_LED_ALERT			XGPIO_DEF2('B', 7)
#define GPIO_LED_MODE			XGPIO_DEF2('C', 12)

