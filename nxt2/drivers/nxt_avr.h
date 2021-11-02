#ifndef __NXT_AVR_H__
#define __NXT_AVR_H__

#include "systypes.h"

#define NXT_AVR_ADDRESS   1
#define NXT_AVR_N_OUTPUTS 4
#define NXT_AVR_N_INPUTS  4

void nxt_avr_init(void);
void nxt_avr_1kHz_update(void);
void nxt_avr_power_down(void);
void nxt_avr_test_loop(void);
void nxt_avr_update(void);

uint32 nxt_avr_get_button_state(void);
uint32 nxt_avr_get_battery_voltage(void);
uint32 nxt_avr_get_sensor_adc(uint8 port);

void nxt_avr_set_motor(uint8 port, sint8 power_percent, uint8 brake);
void nxt_avr_set_input_power(uint8 port, uint32 power_type);

#endif
