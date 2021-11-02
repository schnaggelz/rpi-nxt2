#ifndef __NXT_TOUCH_SENSOR_HPP__
#define __NXT_TOUCH_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
class TouchSensor : public Sensor
{
  public:
    TouchSensor(std::uint8_t port_number)
        : Sensor(port_number)
    {
    }

    bool isPressed();
};
}; // namespace nxt

#endif /* __NXT_TOUCH_SENSOR_HPP__ */
