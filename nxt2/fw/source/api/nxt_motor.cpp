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
    // value cached in C
}

std::int32_t Motor::getCurrentCount()
{
    return nxt_motor_get_current_count(_port_number);
}

std::int32_t Motor::getSpeed()
{
    return nxt_motor_get_speed(_port_number);
}

std::int32_t Motor::getTargetCount()
{
    return nxt_motor_get_current_count(_port_number);
}

void Motor::setCurrentCount(std::int32_t count)
{
    nxt_motor_set_current_count(_port_number, count);
}

void Motor::setTargetCount(std::int32_t count)
{
    nxt_motor_set_target_count(_port_number, count);
}

void Motor::setSpeed(std::int32_t speed)
{
    nxt_motor_set_speed(_port_number, speed, _brake ? 1 : 0);
}

void Motor::rotateTo(std::int32_t angle)
{
    // TODO: Calculate target count from angle
}
} // namespace nxt
