#ifndef _H_TSFUNC_
#define _H_TSFUNC_

extern blm_t			m;
extern pmc_t			pm;

extern void tlm_restart();
extern void sim_runtime(double dT);

int ts_wait_IDLE();
int ts_wait_motion();
int ts_wait_spinup();

void ts_script_default();
void ts_script_base();
void ts_script_test();

#endif /* _H_TSFUNC_ */

