#ifndef _H_DAC_
#define _H_DAC_

#define GPIO_DAC_OUT1			XGPIO_DEF4('A', 4, 4, 5)
#define GPIO_DAC_OUT2			XGPIO_DEF4('A', 5, 5, 5)

enum {
	DAC_OUT1	= 1,
	DAC_OUT2	= 2,
};

void DAC_startup(int mode);
void DAC_halt();

void DAC_set_OUT1(int xOUT);
void DAC_set_OUT2(int xOUT);

#endif /* _H_DAC_ */

