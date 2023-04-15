#ifndef _H_TSFUNC_
#define _H_TSFUNC_

extern blm_t			m;
extern pmc_t			pm;

extern void tlm_restart();
extern void sim_runtime(double dT);

int ts_wait_for_idle();
int ts_wait_for_spinup(float ref);
int ts_wait_for_motion(float ref);

void ts_script_base();
void ts_script_speed();
void ts_script_hfi();
void ts_script_hall();
void ts_script_weakening();
void ts_script_all();

#endif /* _H_TSFUNC_ */

