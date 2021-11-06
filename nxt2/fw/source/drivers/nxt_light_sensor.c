/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C driver code.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "drivers/nxt_light_sensor.h"

#include "drivers/nxt_sensors.h"
#include "drivers/nxt_avr.h"

void nxt_light_sensor_init(uint8 port)
{
    nxt_sensor_set_digi0(port);
}

void nxt_light_sensor_term(uint8 port)
{
    nxt_sensor_unset_digi0(port);
}

sint16 nxt_light_sensor_get_brightness(uint8 port)
{
    return 1023 - nxt_avr_get_sensor_adc(port);
}
