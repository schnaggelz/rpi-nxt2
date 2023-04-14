/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "nxt/nxt_distance_sensor.hpp"

#include "drivers/nxt_sonar_sensor.h"

namespace nxt
{
namespace fw
{
void DistanceSensor::init()
{
    nxt_sonar_sensor_init(_port_number);
}

void DistanceSensor::exit()
{
    nxt_sonar_sensor_term(_port_number);
}

void DistanceSensor::read()
{
    _current_distance = nxt_sonar_sensor_get_distance(_port_number);
}

} // namespace fw
} // namespace nxt
