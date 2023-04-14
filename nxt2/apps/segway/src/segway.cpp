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

#include "utils/maths.h"

#include "nxt/nxt_system.hpp"

namespace nxt
{
namespace segway
{
void Segway::init()
{
    _gyro_sensor.init();
    _gyro_sensor.setMode(nxt::fw::ht::GyroSensorMode::PREVIEW);

    nxt::fw::system::beep(500);

    _motor_angle_index = 0;

    for (auto i = 0; i < NUM_CALIBRATION_CYCLES; i++)
    {
        _gyro_sensor.read();
        _gyro_sensor_raw_value = _gyro_sensor.getAnglarVelocity();

        _gyro_offset +=
            (_gyro_sensor_raw_value - APPROX_SENSOR_OFFSET) * SENSOR_RESOLUTION;

        nxt::fw::system::wait(2);
    }

    _gyro_offset /= NUM_CALIBRATION_CYCLES;

    nxt::fw::system::beep(500);
}

void Segway::sense()
{
    _gyro_sensor.read();
    _gyro_sensor_raw_value = _gyro_sensor.getAnglarVelocity();
}

void Segway::step()
{
    // Get gyro rate
    _gyro_sensor.read();
    _gyro_sensor_raw_value = _gyro_sensor.getAnglarVelocity();

    _gyro_rate =
        (_gyro_sensor_raw_value - APPROX_SENSOR_OFFSET) * SENSOR_RESOLUTION -
        _gyro_offset;

    // Get motor position
    auto motor_angle = -_motor.getCurrentCount();
    _motor_angle_reference = 0;  // update value here
    _motor_angle_error = motor_angle - _motor_angle_reference;

    // Compute motor speed
    _motor_angular_speed =
        (motor_angle - _motor_angle_history[_motor_angle_index]);
    _motor_angular_speed_error = _motor_angular_speed;
    _motor_angle_history[_motor_angle_index] = motor_angle;

    // Compute motor duty cycle
    _motor_duty_cycle =
        (GYRO_ANGLE_GAIN * _gyro_angle_estimated +
         (GYRO_ANGLE_GAIN * _gyro_rate) / 8 +
         MOTOR_ANGLE_GAIN * _motor_angle_error +
         MOTOR_ANGULAR_SPEED_GAIN * _motor_angular_speed_error +
         MOTOR_ANGLE_ERROR_ACCUM_GAIN * _motor_angle_error_accum);

    _motor_duty_cycle = _motor_duty_cycle + sign(_motor_duty_cycle) * 5;

    // Control motors
    // TODO

    // Update angle estimate and gyro offset estimate
    _gyro_angle_estimated =
        _gyro_angle_estimated + _gyro_rate / SENSOR_RESOLUTION;
    _gyro_offset = 0;

    // Update accumulated motor error
    _motor_angle_error_accum = _motor_angle_error_accum + _motor_angle_error;
    _motor_angle_index = (_motor_angle_index + 1) % _motor_angle_history.size();
}

void Segway::exit()
{
    _gyro_sensor.exit();
}
}  // namespace segway
}  // namespace nxt