#define HW_MCU_STM32F722

#define HW_HAVE_ANALOG_KNOB
#define HW_HAVE_BRAKE_KNOB
#define HW_HAVE_STEP_DIR_KNOB
#define HW_HAVE_NTC_ON_PCB
#define HW_HAVE_NTC_MACHINE
#define HW_HAVE_USB_CDC_ACM
#define HW_HAVE_NETWORK_EPCAN
#define HW_HAVE_FAN_CONTROL

#define HW_CLOCK_CRYSTAL_HZ		8000000U

#define HW_PWM_FREQUENCY_HZ		28571.f
#define HW_PWM_DEADTIME_NS		90.f		/* IRF100P218 */

#define HW_PWM_MINIMAL_PULSE		0.2f
#define HW_PWM_CLEARANCE_ZONE		5.0f
#define HW_PWM_SKIP_ZONE		2.0f
#define HW_PWM_BOOTSTRAP_RETENTION	90.f		/* UCC27211A  */

#define HW_ADC_SAMPLING_SEQUENCE	ADC_SEQUENCE__ABC_UTT_TXX

#define HW_ADC_REFERENCE_VOLTAGE	3.3f
#define HW_ADC_SHUNT_RESISTANCE		0.0088f		/* ACS773KCB-150B */
#define HW_ADC_AMPLIFIER_GAIN		1.f

#define HW_ADC_VOLTAGE_R1		800000.f
#define HW_ADC_VOLTAGE_R2		27000.f

#define HW_ADC_KNOB_R1			5000.f
#define HW_ADC_KNOB_R2			10000.f

#define HW_NTC_PCB_TYPE			NTC_LMT87
#define HW_NTC_PCB_BALANCE		0.f
#define HW_NTC_PCB_NTC0			0.f
#define HW_NTC_PCB_TA0			0.f
#define HW_NTC_PCB_BETTA		0.f

#define HW_NTC_EXT_BALANCE		10000.f

#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_CURRENT_C		XGPIO_DEF3('A', 3, 3)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('B', 0, 8)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('C', 0, 10)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('C', 1, 11)
#define GPIO_ADC_NTC_PCB		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_NTC_EXT		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_KNOB_ANG		XGPIO_DEF3('B', 1, 9)
#define GPIO_ADC_KNOB_BRK		XGPIO_DEF3('C', 4, 14)

#define GPIO_STEP			XGPIO_DEF2('B', 6)
#define GPIO_DIR			XGPIO_DEF2('B', 7)

#define GPIO_USART_TX			XGPIO_DEF4('C', 10, 0, 7)
#define GPIO_USART_RX			XGPIO_DEF4('C', 11, 0, 7)

#define GPIO_OTG_FS_DM			XGPIO_DEF4('A', 11, 0, 10)
#define GPIO_OTG_FS_DP			XGPIO_DEF4('A', 12, 0, 10)

#define GPIO_CAN_RX			XGPIO_DEF4('B', 8, 0, 9)
#define GPIO_CAN_TX			XGPIO_DEF4('B', 9, 0, 9)

#define GPIO_GATE_EN			XGPIO_DEF2('B', 2)
#define GPIO_FAN_EN			XGPIO_DEF2('B', 12) | XGPIO_OPEN_DRAIN
#define GPIO_LED_ALERT			XGPIO_DEF2('C', 12)

#define HW_CONFIG_INLINE 	do {						\
					pm.config_IFB = PM_IFB_AB_INLINE;	\
										\
				} while (0)

