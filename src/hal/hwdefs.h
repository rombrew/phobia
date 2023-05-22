#ifndef _H_HWDEFS_
#define _H_HWDEFS_

/* Here is ADC sampling scheme selection macros. Each specified triple of
 * samples is made in parallel on different ADCs.
 *
 * 	"ABC"	- A B and C phase currents
 * 	"U"	- Supply voltage
 * 	"TTT"	- Terminal voltages
 * 	"SC"	- SIN/COS resolver signals
 * 	"XX"	- Not used
 *
 * NOTE: AB and U samples are mandatory. Also we MUST sample all of phase
 * currents at the same time point.
 *
 * */
#define ADC_SEQUENCE__ABU			1
#define ADC_SEQUENCE__ABU_TTT			2
#define ADC_SEQUENCE__ABC_UXX			3
#define ADC_SEQUENCE__ABC_UTT_TXX		4
#define ADC_SEQUENCE__ABC_UTT_TSC		5

#define ADC_HAVE_SEQUENCE__ABC(m)		((m) == 3 || (m) == 4 || (m) == 5)
#define ADC_HAVE_SEQUENCE__TTT(m)		((m) == 2 || (m) == 4 || (m) == 5)

#ifndef _HW_REV
#error HW revision macro _HW_REV must be defined
#endif /* _HW_REV */

#ifndef _HW_INCLUDE
#error HW revision macro _HW_INCLUDE must be defined
#endif /* _HW_INCLUDE */

#include "gpio.h"

/* We include appropriate configuration file according to the hardware revision
 * selected in make options.
 * */
#include _HW_INCLUDE

#endif /* _H_HWDEFS_ */

