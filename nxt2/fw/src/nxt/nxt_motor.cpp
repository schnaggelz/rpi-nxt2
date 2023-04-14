/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "nxt/nxt_motor.hpp"

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
}

void Motor::exit() noexcept
{
    nxt_motor_set_speed(_port_number, 0, 0);
}

void Motor::read() noexcept
{
    // nothing to do
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

void Motor::setTargetCount(std::int32_t count, std::int32_t tolerance) noexcept
{
    nxt_motor_set_target_count(_port_number, count, tolerance);
}

void Motor::stop() noexcept
{
    nxt_motor_set_speed(_port_number, 0, 1);
}

void Motor::setSpeed(std::int32_t speed) noexcept
{
    nxt_motor_set_speed(_port_number, speed, _brake ? 1 : 0);
}

} // namespace fw
} // namespace nxt
