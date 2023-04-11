/*******************************************************************************
 * Copyright (C) 2023 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Segway demo application.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "segway.hpp"

namespace nxt
{
namespace segway
{
void Segway::init()
{
    _gyro_sensor.init();
    _motor_angle_index = 0;

    for(auto i = 0; i < NUM_CALIBRATION_CYCLES; i++)
    {
        _gyro_offset += (_gyro_sensor.getAnglarVelocity() - APPROX_SENSOR_OFFSET) * SENSOR_RESOLUTION;
        
        nxt::fw::system::wait(2);
    }

    _gyro_offset /= NUM_CALIBRATION_CYCLES;
}

void Segway::step() {}

void Segway::exit() 
{
    _gyro_sensor.exit();
}
} // namespace segway
} // namespace nxt