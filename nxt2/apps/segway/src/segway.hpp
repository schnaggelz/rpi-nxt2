/*******************************************************************************
 * Copyright (C) 2023 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Segway demo application.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_RC_SEGWAY_HPP__
#define __NXT_RC_SEGWAY_HPP__

#include "nxt/nxt_ht_gyro_sensor.hpp"
#include "nxt/nxt_motor.hpp"
#include "nxt/nxt_system.hpp"

#include <array>
#include <cstdint>

namespace nxt
{
namespace segway
{
class Segway
{
  public:
    Segway(std::uint16_t sample_period_ms)
        : _sample_period_ms(sample_period_ms)
        , _motors({nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_A),
                   nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_B)})
        , _gyro_sensor(nxt::fw::Sensor::Port::PORT_1)
    {
    }

    ~Segway() = default;

    void init();
    void step();
    void exit();

    void setSpeed(std::uint16_t speed)
    {
        _speed = speed;
    }

    void setDirection(std::uint16_t direction)
    {
        _direction = direction;
    }

    static constexpr std::uint16_t SENSOR_RESOLUTION = 16;
    static constexpr std::uint16_t APPROX_SENSOR_OFFSET = 585;
    static constexpr std::uint16_t NUM_CALIBRATION_CYCLES = 50;

  private:
    std::uint16_t _sample_period_ms;
    std::array<nxt::fw::Motor, 2> _motors;
    nxt::fw::ht::GyroSensor _gyro_sensor;

    std::uint16_t _speed{0};
    std::uint16_t _direction{0};

    std::array<std::int32_t, 5> _motor_angle_history{0};
    std::uint8_t _motor_angle_index{0};

    std::int32_t _motor_angle_reference{0};
    std::int32_t _motor_angle_error{0};
    std::int32_t _motor_angle_error_accum{0};
    std::int32_t _motor_angular_speed{0};
    std::int32_t _motor_angular_speed_error{0};
    std::int32_t _motor_duty_cycle{0};

    std::int32_t _gyro_rate{0};
    std::int32_t _gyro_offset{0};
    std::int32_t _gyro_angle_estimated{0};
};

} // namespace segway
} // namespace nxt

#endif /* __NXT_RC_SEGWAY_HPP__ */
