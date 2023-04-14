/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "nxt/nxt_ht_gyro_sensor.hpp"

#include "drivers/nxt_ht_gyro_sensor.h"

namespace nxt
{
namespace fw
{
namespace ht
{
void GyroSensor::init()
{
    nxt_ht_gyro_sensor_init(_port_number);
}

void GyroSensor::exit()
{
    nxt_ht_gyro_sensor_term(_port_number);
}

void GyroSensor::read()
{
    if (_mode == GyroSensorMode::PREVIEW)
    {
        _angular_velocity =
            nxt_ht_gyro_sensor_get_angular_velocity_preview(_port_number);
    }
    else
    {
        _angular_velocity =
            nxt_ht_gyro_sensor_get_angular_velocity(_port_number);
    }
}

}  // namespace ht
}  // namespace fw
}  // namespace nxt
