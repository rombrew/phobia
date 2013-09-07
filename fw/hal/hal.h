#ifndef _H_HAL_
#define _H_HAL_

/* Include HAL-specific inline routines.
 * */
#include HALINL

int hal_exclusive_load(void *addr);
int hal_exclusive_store(void *addr, int val);
void hal_memory_barrier();

void hal_rt_irq_enable();
void hal_rt_irq_disable();
void hal_rt_irq_request();

void rt_tick();
void rt_schedule();

void *hal_stack_init(void *sp, void *entry);
void hal_cpu_relax();

int hal_reset_reason();

#endif /* _H_HAL_ */

