/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "api/nxt_motor.hpp"

#include "drivers/nxt_motors.h"

namespace nxt
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
    _current_count = nxt_motor_get_count(_port_number);
    _current_speed = nxt_motor_get_speed(_port_number);
}

std::int32_t Motor::getCount()
{
    _current_count = nxt_motor_get_count(_port_number);

    return _current_count;
}

std::int32_t Motor::getSpeed()
{
    _current_speed = nxt_motor_get_speed(_port_number);

    return _current_count;
}

void Motor::setCount(std::int32_t count)
{
    _current_count = count;

    nxt_motor_set_count(_port_number, _current_count);
}

void Motor::setSpeed(std::int32_t speed)
{
    _current_speed = speed;

    nxt_motor_set_speed(_port_number, _current_speed, _brake ? 1 : 0);
}

void Motor::rotateTo(std::int32_t angle)
{
    // TODO: Calculate target count from angle
    _target_count = angle;

    nxt_motor_command(_port_number, _target_count, _current_speed);
}
} // namespace nxt
