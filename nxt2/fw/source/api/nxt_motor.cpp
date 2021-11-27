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
void Motor::init()
{
    nxt_motor_set_speed(_port_number, 0, _brake ? 1 : 0);
}

void Motor::exit()
{
    nxt_motor_set_speed(_port_number, 0, 0);
}

void Motor::read()
{
    if (!_target_reached)
    {
        const auto count = nxt_motor_get_current_count(_port_number);
        const auto speed = nxt_motor_get_speed(_port_number);

        if (speed != 0)
        {
            if (speed > 0 && count >= _target_count)
            {
                nxt_motor_set_speed(_port_number, 0, 1);
                _target_reached = true;
            }
            else if (speed < 0 && count <= _target_count)
            {
                nxt_motor_set_speed(_port_number, 0, 1);
                _target_reached = true;
            }
        }
    }
}

std::int32_t Motor::getCurrentCount() const
{
    return nxt_motor_get_current_count(_port_number);
}

std::int32_t Motor::getSpeed() const
{
    return nxt_motor_get_speed(_port_number);
}

std::int32_t Motor::getTargetCount() const
{
    return nxt_motor_get_current_count(_port_number);
}

void Motor::setCurrentCount(std::int32_t count)
{
    nxt_motor_set_current_count(_port_number, count);
}

void Motor::setTargetCount(std::int32_t count)
{
    _target_count = count;
    _target_reached = false;
}

void Motor::resetTarget()
{
    _target_count = std::numeric_limits<decltype(_target_count)>::max();
    _target_reached = true;
}

void Motor::setSpeed(std::int32_t speed)
{
    nxt_motor_set_speed(_port_number, speed, _brake ? 1 : 0);
}

} // namespace fw
} // namespace nxt
