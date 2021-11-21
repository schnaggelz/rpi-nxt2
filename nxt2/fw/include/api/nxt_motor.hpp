/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_SENSING_MOTOR_HPP__
#define __NXT_SENSING_MOTOR_HPP__

#include "nxt_actuator.hpp"

namespace nxt
{
class Motor : public Actuator
{
  public:
    Motor(Port port, bool brake)
        : Actuator(port)
        , _brake(brake)
    {
    }

    ~Motor() = default;

    void setSpeed(std::int32_t speed);
    void setCurrentCount(std::int32_t count);
    void setTargetCount(std::int32_t count);
    void rotateTo(std::int32_t angle);

    std::int32_t getSpeed();
    std::int32_t getCurrentCount();
    std::int32_t getTargetCount();

    void init() override;
    void read() override;
    void exit() override;

  private:
    bool _brake;
};
} // namespace nxt

#endif /* __NXT_SENSING_MOTOR_HPP__ */