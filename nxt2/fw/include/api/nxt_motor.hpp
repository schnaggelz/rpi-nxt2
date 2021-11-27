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
namespace fw
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

    void setSpeed(std::int32_t speed) noexcept;
    void setCurrentCount(std::int32_t count) noexcept;
    void setTargetCount(std::int32_t count) noexcept;

    std::int32_t getSpeed() const noexcept;
    std::int32_t getCurrentCount() const noexcept;
    std::int32_t getTargetCount() const noexcept;

    void init() noexcept override;
    void read() noexcept override;
    void exit() noexcept override;

  private:
    bool _brake{false};
    bool _target_reached{false};
    std::int32_t _target_count{0};
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_SENSING_MOTOR_HPP__ */