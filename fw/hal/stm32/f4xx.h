#ifndef _H_F4XX_
#define _H_F4XX_

#include "lib/types.h"

#define __I			volatile const
#define __IO			volatile

#define BIT(n)			(1UL << (n))
#define BITF(bf, n)		((bf) << (n))

typedef struct
{
	__IO u32_t	CTRL;
	__IO u32_t	LOAD;
	__IO u32_t	VAL;
	__I u32_t	CALIB;

} stm32_stk_t;

typedef struct
{
	__IO u32_t	ISER[8];
	u32_t		RESERVED1[24];
	__IO u32_t	ICER[8];
	u32_t		RESERVED2[24];
	__IO u32_t	ISPR[8];
	u32_t		RESERVED3[24];
	__IO u32_t	ICPR[8];
	u32_t		RESERVED4[24];
	__IO u32_t	IABR[8];
	u32_t		RESERVED5[56];
	__IO u8_t	IP[240];
	u32_t		RESERVED6[644];
	__IO u32_t	STIR;

} stm32_nvic_t;

typedef struct
{
	__I u32_t	CPUID;
	__IO u32_t	ICSR;
	__IO u32_t	VTOR;
	__IO u32_t	AIRCR;
	__IO u32_t	SCR;
	__IO u32_t	CCR;
	__IO u8_t	SHP[12];
	__IO u32_t	SHCSR;
	__IO u32_t	CFSR;
	__IO u32_t	HFSR;
	__IO u32_t	DFSR;
	__IO u32_t	MMFAR;
	__IO u32_t	BFAR;
	__IO u32_t	AFSR;
	__I u32_t	PFR[2];
	__I u32_t	DFR;
	__I u32_t	ADR;
	__I u32_t	MMFR[4];
	__I u32_t	ISAR[5];
	u32_t		RESERVED1[5];
	__IO u32_t	CPACR;

} stm32_scb_t;

typedef struct
{
	__IO u32_t	TYPE;
	__IO u32_t	CTRL;
	__IO u32_t	RNR;
	__IO u32_t	RBAR;
	__IO u32_t	RASR;
	__IO u32_t	RBAR_A1;
	__IO u32_t	RASR_A1;
	__IO u32_t	RBAR_A2;
	__IO u32_t	RASR_A2;
	__IO u32_t	RBAR_A3;
	__IO u32_t	RASR_A3;

} stm32_mpu_t;

typedef struct
{
	u32_t		RESERVED0[1];
	__IO u32_t	FPCCR;
	__IO u32_t	FPCAR;
	__IO u32_t	FPDSCR;
	__I  u32_t	MVFR0;
	__I  u32_t	MVFR1;

} stm32_fpu_t;

typedef struct
{
	__IO u32_t	ACR;
	__IO u32_t	KEYR;
	__IO u32_t	OPTKEYR;
	__IO u32_t	SR;
	__IO u32_t	CR;
	__IO u32_t	OPTCR;
	__IO u32_t	OPTCR1;

} stm32_flash_t;

typedef struct
{
	__IO u32_t	MODER;
	__IO u32_t	OTYPER;
	__IO u32_t	OSPEEDR;
	__IO u32_t	PUPDR;
	__IO u32_t	IDR;
	__IO u32_t	ODR;
	__IO u16_t	BSRRL;
	__IO u16_t	BSRRH;
	__IO u32_t	LCKR;
	__IO u32_t	AFR[2];

} stm32_gpio_t;

typedef struct
{
	__IO u32_t	MEMRMP;
	__IO u32_t	PMC;
	__IO u32_t	EXTICR[4];
	u32_t		RESERVED[2];
	__IO u32_t	CMPCR;

} stm32_syscfg_t;

typedef struct
{
	__IO u32_t	KR;
	__IO u32_t	PR;
	__IO u32_t	RLR;
	__IO u32_t	SR;

} stm32_iwdg_t;

typedef struct
{
	__IO u32_t	CR;
	__IO u32_t	CSR;

} stm32_pwr_t;

typedef struct
{
	__IO u32_t	CR;
	__IO u32_t	PLLCFGR;
	__IO u32_t	CFGR;
	__IO u32_t	CIR;
	__IO u32_t	AHB1RSTR;
	__IO u32_t	AHB2RSTR;
	__IO u32_t	AHB3RSTR;
	u32_t     	RESERVED0;
	__IO u32_t	APB1RSTR;
	__IO u32_t	APB2RSTR;
	u32_t     	RESERVED1[2];
	__IO u32_t	AHB1ENR;
	__IO u32_t	AHB2ENR;
	__IO u32_t	AHB3ENR;
	u32_t     	RESERVED2;
	__IO u32_t	APB1ENR;
	__IO u32_t	APB2ENR;
	u32_t     	RESERVED3[2];
	__IO u32_t	AHB1LPENR;
	__IO u32_t	AHB2LPENR;
	__IO u32_t	AHB3LPENR;
	u32_t     	RESERVED4;
	__IO u32_t	APB1LPENR;
	__IO u32_t	APB2LPENR;
	u32_t     	RESERVED5[2];
	__IO u32_t	BDCR;
	__IO u32_t	CSR;
	u32_t     	RESERVED6[2];
	__IO u32_t	SSCGR;
	__IO u32_t	PLLI2SCFGR;

} stm32_rcc_t;

typedef struct
{
	__IO u32_t	POWER;
	__IO u32_t	CLKCR;
	__IO u32_t	ARG;
	__IO u32_t	CMD;
	__I u32_t 	RESPCMD;
	__I u32_t 	RESP1;
	__I u32_t 	RESP2;
	__I u32_t 	RESP3;
	__I u32_t 	RESP4;
	__IO u32_t	DTIMER;
	__IO u32_t	DLEN;
	__IO u32_t	DCTRL;
	__I u32_t 	DCOUNT;
	__I u32_t 	STA;
	__IO u32_t	ICR;
	__IO u32_t	MASK;
	u32_t     	RESERVED0[2];
	__I u32_t 	FIFOCNT;
	u32_t     	RESERVED1[13];
	__IO u32_t	FIFO;

} stm32_sdio_t;

typedef struct
{
	__IO u16_t	CR1;
	u16_t     	RESERVED0;
	__IO u16_t	CR2;
	u16_t     	RESERVED1;
	__IO u16_t	SMCR;
	u16_t     	RESERVED2;
	__IO u16_t	DIER;
	u16_t     	RESERVED3;
	__IO u16_t	SR;
	u16_t     	RESERVED4;
	__IO u16_t	EGR;
	u16_t     	RESERVED5;
	__IO u16_t	CCMR1;
	u16_t     	RESERVED6;
	__IO u16_t	CCMR2;
	u16_t     	RESERVED7;
	__IO u16_t	CCER;
	u16_t     	RESERVED8;
	__IO u32_t	CNT;
	__IO u16_t	PSC;
	u16_t     	RESERVED9;
	__IO u32_t	ARR;
	__IO u16_t	RCR;
	u16_t     	RESERVED10;
	__IO u32_t	CCR1;
	__IO u32_t	CCR2;
	__IO u32_t	CCR3;
	__IO u32_t	CCR4;
	__IO u16_t	BDTR;
	u16_t     	RESERVED11;
	__IO u16_t	DCR;
	u16_t     	RESERVED12;
	__IO u16_t	DMAR;
	u16_t     	RESERVED13;
	__IO u16_t	OR;
	u16_t     	RESERVED14;

} stm32_tim_t;

typedef struct
{
	__IO u16_t	CR1;
	u16_t     	RESERVED0;
	__IO u16_t	CR2;
	u16_t     	RESERVED1;
	__IO u16_t	SR;
	u16_t     	RESERVED2;
	__IO u16_t	DR;
	u16_t     	RESERVED3;
	__IO u16_t	CRCPR;
	u16_t     	RESERVED4;
	__IO u16_t	RXCRCR;
	u16_t     	RESERVED5;
	__IO u16_t	TXCRCR;
	u16_t     	RESERVED6;
	__IO u16_t	I2SCFGR;
	u16_t     	RESERVED7;
	__IO u16_t	I2SPR;
	u16_t     	RESERVED8;

} stm32_spi_t;

typedef struct
{
	__IO u16_t	SR;
	u16_t     	RESERVED0;
	__IO u16_t	DR;
	u16_t     	RESERVED1;
	__IO u16_t	BRR;
	u16_t     	RESERVED2;
	__IO u16_t	CR1;
	u16_t     	RESERVED3;
	__IO u16_t	CR2;
	u16_t     	RESERVED4;
	__IO u16_t	CR3;
	u16_t     	RESERVED5;
	__IO u16_t	GTPR;
	u16_t     	RESERVED6;

} stm32_usart_t;

typedef struct
{
	__IO u32_t	CR;
	__IO u32_t	CFR;
	__IO u32_t	SR;

} stm32_wwdg_t;

typedef struct 
{
	__IO u32_t	CR;
	__IO u32_t	SR;
	__IO u32_t	DR;

} stm32_rng_t;

typedef struct
{
	__IO u32_t	SR;
	__IO u32_t	CR1;
	__IO u32_t	CR2;
	__IO u32_t	SMPR1;
	__IO u32_t	SMPR2;
	__IO u32_t	JOFR1;
	__IO u32_t	JOFR2;
	__IO u32_t	JOFR3;
	__IO u32_t	JOFR4;
	__IO u32_t	HTR;
	__IO u32_t	LTR;
	__IO u32_t	SQR1;
	__IO u32_t	SQR2;
	__IO u32_t	SQR3;
	__IO u32_t	JSQR;
	__IO u32_t	JDR1;
	__IO u32_t	JDR2;
	__IO u32_t	JDR3;
	__IO u32_t	JDR4;
	__IO u32_t	DR;

} stm32_adc_t;

typedef struct
{
	__IO u32_t	CSR;
	__IO u32_t	CCR;
	__IO u32_t	CDR;

} stm32_common_adc_t;

typedef struct
{
	__IO u32_t	CR;
	__IO u32_t	SWTRIGR;
	__IO u32_t	DHR12R1;
	__IO u32_t	DHR12L1;
	__IO u32_t	DHR8R1;
	__IO u32_t	DHR12R2;
	__IO u32_t	DHR12L2;
	__IO u32_t	DHR8R2;
	__IO u32_t	DHR12RD;
	__IO u32_t	DHR12LD;
	__IO u32_t	DHR8RD;
	__IO u32_t	DOR1;
	__IO u32_t	DOR2;
	__IO u32_t	SR;

} stm32_dac_t;

typedef struct
{
	__IO u32_t	CR;
	__IO u32_t	NDTR;
	__IO u32_t	PAR;
	__IO u32_t	M0AR;
	__IO u32_t	M1AR;
	__IO u32_t	FCR;

} stm32_dma_stream_t;

typedef struct
{
	__IO u32_t	LISR;
	__IO u32_t	HISR;
	__IO u32_t	LIFCR;
	__IO u32_t	HIFCR;

	stm32_dma_stream_t	S[8];

} stm32_dma_t;

#define STK			((stm32_stk_t *) 0xE000E010)
#define NVIC			((stm32_nvic_t *) 0xE000E100)
#define SCB			((stm32_scb_t *) 0xE000ED00)
#define MPU			((stm32_mpu_t *) 0xE000ED90)
#define FPU			((stm32_fpu_t *) 0xE000EF30)

#define TIM2			((stm32_tim_t *) 0x40000000)
#define TIM3			((stm32_tim_t *) 0x40000400)
#define TIM4			((stm32_tim_t *) 0x40000800)
#define TIM5			((stm32_tim_t *) 0x40000C00)
#define TIM6			((stm32_tim_t *) 0x40001000)
#define TIM7			((stm32_tim_t *) 0x40001400)
#define TIM12			((stm32_tim_t *) 0x40001800)
#define TIM13			((stm32_tim_t *) 0x40001C00)
#define TIM14			((stm32_tim_t *) 0x40002000)

#define WWDG			((stm32_wwdg_t *) 0x40002C00)
#define IWDG			((stm32_iwdg_t *) 0x40003000)
#define SPI2			((stm32_spi_t *) 0x40003800)
#define SPI3			((stm32_spi_t *) 0x40003C00)
#define USART2			((stm32_usart_t *) 0x40004400)
#define USART3			((stm32_usart_t *) 0x40004800)
#define UART4			((stm32_usart_t *) 0x40004C00)
#define UART5			((stm32_usart_t *) 0x40005000)
#define PWR			((stm32_pwr_t *) 0x40007000)
#define DAC			((stm32_dac_t *) 0x40007400)
#define UART7			((stm32_usart_t *) 0x40007800)
#define UART8			((stm32_usart_t *) 0x40007C00)

#define TIM1			((stm32_tim_t *) 0x40010000)
#define TIM8			((stm32_tim_t *) 0x40010400)
#define USART1			((stm32_uart_t *) 0x40011000)
#define USART6			((stm32_uart_t *) 0x40011400)
#define ADC1			((stm32_adc_t *) 0x40012000)
#define ADC2			((stm32_adc_t *) 0x40012100)
#define ADC3			((stm32_adc_t *) 0x40012200)
#define ADC			((stm32_common_adc_t *) 0x40012300)
#define SDIO			((stm32_sdio_t *) 0x40012C00)
#define SPI1			((stm32_spi_t *) 0x40013000)
#define SPI4			((stm32_spi_t *) 0x40013400)
#define SYSCFG			((stm32_syscfg_t *) 0x40013800)
#define TIM9			((stm32_tim_t *) 0x40014000)
#define TIM10			((stm32_tim_t *) 0x40014400)
#define TIM11			((stm32_tim_t *) 0x40014800)
#define SPI5			((stm32_spi_t *) 0x40015000)
#define SPI6			((stm32_spi_t *) 0x40015400)

#define GPIOA			((stm32_gpio_t *) 0x40020000)
#define GPIOB			((stm32_gpio_t *) 0x40020400)
#define GPIOC			((stm32_gpio_t *) 0x40020800)
#define GPIOD			((stm32_gpio_t *) 0x40020C00)
#define GPIOE			((stm32_gpio_t *) 0x40021000)
#define GPIOF			((stm32_gpio_t *) 0x40021400)
#define GPIOG			((stm32_gpio_t *) 0x40021800)
#define GPIOH			((stm32_gpio_t *) 0x40021C00)
#define GPIOI			((stm32_gpio_t *) 0x40022000)

#define RCC			((stm32_rcc_t *) 0x40023800)
#define FLASH			((stm32_flash_t *) 0x40023C00)
#define DMA1			((stm32_dma_t *) 0x40026000)
#define DMA2			((stm32_dma_t *) 0x40026400)

#define RNG			((stm32_rng_t *) 0x50060800)

#endif /* _H_F4XX_ */

