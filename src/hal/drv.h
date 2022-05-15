#ifndef _H_DRV_
#define _H_DRV_

enum {
	DRV_NONE		= 0,
	DRV_PART_DRV8303,
	DRV_PART_DRV8305
};

typedef struct {

	int		part;
	int		auto_RESET;

	int		gpio_GATE_EN;
	int		gpio_FAULT;

	int		status_raw;
	int		gate_current;
	int		ocp_level;

	int		device_ON;
}
DRV_config_t;

void DRV_startup();
void DRV_halt();

void DRV_configure();
void DRV_status();
int DRV_fault();

#endif /* _H_DRV_ */

