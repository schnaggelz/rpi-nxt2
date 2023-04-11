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

#include "api/nxt_motor.hpp"

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

  private:
    std::uint16_t _sample_period_ms;
    std::array<nxt::fw::Motor, 2> _motors;

    std::uint16_t _speed{0};
    std::uint16_t _direction{0};

    std::array<std::int32_t, 5> _motor_angle_history{0};
};

} // namespace segway
} // namespace nxt

#endif /* __NXT_RC_SEGWAY_HPP__ */
