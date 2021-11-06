/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_SENSING_MOTOR_HPP__
#define __NXT_SENSING_MOTOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
class SensingMotor : public Sensor
{
  public:
    SensingMotor(std::uint8_t port_number, bool brake)
        : Sensor(port_number), _current_speed(0), _brake(brake)
    {
    }

    void setSpeed(std::int32_t speed);

    void setCount(std::int32_t count);

    void rotateTo(std::int32_t angle);

    std::int32_t getCount();

    std::int32_t getSpeed();

    void init() override;
    void read() override;
    void exit() override;

  private:
    std::int32_t _current_speed{0};
    std::int32_t _current_count{0};
    std::int32_t _target_count{0};

    bool _brake;
};
} // namespace nxt

#endif /* __NXT_SENSING_MOTOR_HPP__ */