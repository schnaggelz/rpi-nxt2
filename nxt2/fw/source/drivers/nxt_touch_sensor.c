/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C driver code.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "drivers/nxt_touch_sensor.h"

#include "drivers/nxt_avr.h"

uint8 nxt_touch_sensor_is_pressed(uint8 port)
{
    return (nxt_avr_get_sensor_adc(port) < 512);
}
