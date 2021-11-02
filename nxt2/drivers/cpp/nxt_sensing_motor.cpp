#include "nxt_sensing_motor.hpp"

#include "nxt_motors.h"

using namespace nxt;

void SensingMotor::init()
{
    nxt_motor_set_speed(_port_number, 0, _brake ? 1 : 0);
}

int32_t SensingMotor::getCount() const
{
    return nxt_motor_get_count(_port_number);
}

void SensingMotor::setCount(int32_t count)
{
    nxt_motor_set_count(_port_number, count);
}

void SensingMotor::setSpeed(int32_t speed)
{
    _current_speed = speed;

    nxt_motor_set_speed(_port_number, speed, _brake ? 1 : 0);
}
