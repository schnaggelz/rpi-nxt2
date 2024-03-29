/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "nxt/nxt_ht_compass_sensor.hpp"

#include "drivers/nxt_ht_compass_sensor.h"

namespace nxt
{
namespace fw
{
namespace ht
{
void CompassSensor::init()
{
    nxt_ht_compass_sensor_init(_port_number);
}

void CompassSensor::exit()
{
    nxt_ht_compass_sensor_term(_port_number);
}

bool CompassSensor::calibrate()
{
    return nxt_ht_compass_sensor_calibrate(_port_number) > 0 ? false : true;
}

void CompassSensor::read()
{
    _heading = nxt_ht_compass_sensor_get_direction(_port_number);
}

} // namespace ht
} // namespace fw
} // namespace nxt
