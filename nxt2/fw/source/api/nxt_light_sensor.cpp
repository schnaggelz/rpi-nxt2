/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "api/nxt_light_sensor.hpp"

#include "drivers/nxt_light_sensor.h"

namespace nxt
{
void LightSensor::init()
{
    nxt_light_sensor_init(_port_number);
}

void LightSensor::exit()
{
    nxt_light_sensor_term(_port_number);
}

void LightSensor::read()
{
    _brightness = nxt_light_sensor_get_brightness(_port_number);
}
} // namespace nxt