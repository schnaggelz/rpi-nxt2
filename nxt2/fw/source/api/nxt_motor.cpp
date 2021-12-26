/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "api/nxt_motor.hpp"

#include "drivers/nxt_motors.h"

#include <limits>

namespace nxt
{
namespace fw
{
void Motor::init() noexcept
{
    nxt_motor_set_current_count(_port_number, 0);
    nxt_motor_set_speed(_port_number, 0, _brake ? 1 : 0);

    _target_reached = true;
}

void Motor::exit() noexcept
{
    nxt_motor_set_speed(_port_number, 0, 0);
}

void Motor::read() noexcept
{
    const auto speed = nxt_motor_get_speed(_port_number);

    if (!_target_reached && speed != 0)
    {
        const auto current_count = nxt_motor_get_current_count(_port_number);
        const auto target_count = nxt_motor_get_target_count(_port_number);

        if (target_count < current_count - TOLERANCE)
        {
            const auto expected_speed = -std::abs(speed);
            if (speed != expected_speed)
            {
                nxt_motor_set_speed(_port_number, expected_speed, 1);
            }
        }
        else if (target_count > current_count + TOLERANCE)
        {
            const auto expected_speed = std::abs(speed);
            if (speed != expected_speed)
            {
                nxt_motor_set_speed(_port_number, expected_speed, 1);
            }
        }
        else
        {
            nxt_motor_set_speed(_port_number, 0, 1);

            _target_reached = true;
        }
    }
}

std::int32_t Motor::getCurrentCount() const noexcept
{
    return nxt_motor_get_current_count(_port_number);
}

std::int32_t Motor::getSpeed() const noexcept
{
    return nxt_motor_get_speed(_port_number);
}

std::int32_t Motor::getTargetCount() const noexcept
{
    return nxt_motor_get_target_count(_port_number);
}

void Motor::setCurrentCount(std::int32_t count) noexcept
{
    nxt_motor_set_current_count(_port_number, count);
}

void Motor::setTargetCount(std::int32_t count) noexcept
{
    nxt_motor_set_target_count(_port_number, count);

    _target_reached = false;
}

void Motor::stop() noexcept
{
    nxt_motor_set_speed(_port_number, 0, 1);

    _target_reached = true;
}

void Motor::setSpeed(std::int32_t speed) noexcept
{
    nxt_motor_set_speed(_port_number, speed, _brake ? 1 : 0);
}

} // namespace fw
} // namespace nxt
