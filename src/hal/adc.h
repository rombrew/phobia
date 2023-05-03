#ifndef _H_ADC_
#define _H_ADC_

#define ADC_RESOLUTION			4096

enum {
	ADC_SMP_3	= 0,	/* ~ 0.07 (us) */
	ADC_SMP_15,		/* ~ 0.35 (us) */
	ADC_SMP_28,		/* ~ 0.66 (us) */
	ADC_SMP_56,		/* ~ 1.33 (us) */
	ADC_SMP_84,		/* ~ 2.00 (us) */
	ADC_SMP_112,		/* ~ 2.66 (us) */
	ADC_SMP_144,		/* ~ 3.42 (us) */
	ADC_SMP_480		/* ~ 11.4 (us) */
};

void ADC_const_build();
void ADC_startup();

float ADC_get_VALUE(int xGPIO);

extern void ADC_IRQ();

#endif /* _H_ADC_ */

