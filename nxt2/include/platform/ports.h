/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __PORTS_H__
#define __PORTS_H__

#include "systypes.h"

#define NUM_SENSOR_PORTS (4)
#define RS485_PORT (3)

/* Sensor port pin modes */
#define SP_MODE_OFF 0
#define SP_MODE_INPUT 1
#define SP_MODE_OUTPUT 2
#define SP_MODE_ADC 3

#define SP_DIGI0 0
#define SP_DIGI1 1
#define SP_ANA 2

typedef struct
{
    int pins[2];
    unsigned long adc_channel;
    volatile unsigned int* adc_data;
}
port_pins;

extern const port_pins sensor_pins[];

extern void sp_init(void);
extern void sp_set(uint8 port, uint8 pin, sint32 val);
extern void sp_reset(uint8 port);
extern sint32 sp_get(uint8 port, uint8 pin);
extern void sp_set_mode(uint8 port, uint8 pin, sint32 mode);
extern sint32 sp_read(uint8 port, uint8 pin);
extern void sp_set_power(uint8 port, uint32 val);
extern sint32 sp_check_event(sint32 filter);

#endif // __PORTS_H__
