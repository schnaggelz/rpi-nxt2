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
std::uint16_t GyroSensor::getAnglarVelocity(bool preview) const noexcept
{
    if (preview)
    {
        return _angular_velocity_preview;
    }
    else
    {
        return _angular_velocity;
    }
}

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
    _angular_velocity = nxt_ht_gyro_sensor_get_angular_velocity(_port_number);
    _angular_velocity_preview = nxt_ht_gyro_sensor_get_angular_velocity_preview(_port_number);
}

} // namespace ht
} // namespace fw
} // namespace nxt
