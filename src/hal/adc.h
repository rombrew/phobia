#ifndef _H_ADC_
#define _H_ADC_

#define ADC_RESOLUTION			4096

void ADC_startup();
void ADC_configure();
float ADC_get_VALUE(int xGPIO);

extern void ADC_IRQ();

#endif /* _H_ADC_ */

