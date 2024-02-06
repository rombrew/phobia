#ifndef _H_DRV_
#define _H_DRV_

enum {
	DRV_NONE		= 0,
	DRV_PART_DRV8301,
	DRV_PART_DRV8305
};

typedef struct {

	int		partno;

	int		gpio_GATE_EN;
	int		gpio_FAULT;

	int		auto_RESTART;
	int		status_raw;
	int		gate_current;
	int		ocp_level;

	int		partno_ENABLED;
	int		fault_CNT;
}
DRV_config_t;

void DRV_startup();
void DRV_halt();

void DRV_configure();
void DRV_status();

int DRV_fault();

float DRV_gate_current();
float DRV_ocp_level();

#endif /* _H_DRV_ */

