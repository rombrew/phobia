#ifndef _H_HAL_
#define _H_HAL_

/* Include hardware specific inline routines.
 * */
#include HWINL

int hal_reset_reason();

void irq_startup();
void irq_systick();
void irq_bridge(const int adc[2]);
void irq_network(unsigned char pkg[10]);
void irq_serial(int sym);
void irq_worker();

void hal_irq_mask(int mask);

void hal_bridge_config(float freq);
void hal_bridge_dc(const float dc[3]);
void hal_network_config(int baud);
void hal_serial_config(int baud);
void hal_serial_send(int sym);
void hal_worker_request();

void hal_adc_read(const int adc[4]);
void hal_led_ctl(int i);

#endif /* _H_HAL_ */

