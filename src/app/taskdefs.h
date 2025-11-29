#ifndef _H_TASKDEFS_
#define _H_TASKDEFS_

#define AP_TASK_PRIORITY	1	/* quite low priority for all applications. */

#define AP_TASK_DEF(name)	LD_TASK void app_ ## name (void *pData)
#define AP_KNOB(knob)		volatile int * (knob) = (volatile int *) pData;

#define AP_CONDITION(knob)	(* (knob) != 0)
#define AP_TERMINATE(knob)	do { * (knob) = 0; vTaskDelete(NULL); } while (0)

#endif /* _H_TASKDEFS_ */

