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

    std::int32_t getCount() const;

    std::int32_t getSpeed() const
    {
        return _current_speed;
    }

    void init() override;
    void read() override;
    void exit() override;

  private:
    std::int32_t _current_speed;

    bool _brake;
};
} // namespace nxt

#endif /* __NXT_SENSING_MOTOR_HPP__ */