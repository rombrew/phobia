#ifndef _H_DRV_
#define _H_DRV_

#ifndef HW_DRV_FAULT_SAFETY
#define HW_DRV_FAULT_SAFETY		40
#endif /* HW_DRV_FAULT_SAFETY */

enum {
	DRV_NONE		= 0,
	DRV_PART_DRV8301,
	DRV_PART_DRV8305
};

typedef struct {

	int		part;

	int		gpio_GATE_EN;
	int		gpio_FAULT;

	int		auto_RESTART;
	int		status_raw;
	int		gate_current;
	int		ocp_level;
	int		fault_safety;

	int		gate_ON;
	int		fault_CNT;
}
DRV_config_t;

void DRV_startup();
void DRV_halt();

void DRV_configure();
void DRV_status();
int DRV_fault();

#endif /* _H_DRV_ */

