#ifndef __NXT_TOUCH_SENSOR_HPP__
#define __NXT_TOUCH_SENSOR_HPP__

#include <stdint.h>

#include "../../../source/drivers/cpp/nxt_sensor.hpp"

namespace nxt
{
    class TouchSensor : public Sensor
    {
    public:
        TouchSensor(uint8_t port_number)
            : Sensor(port_number)
        {
        }

        bool isPressed();
    };
};

#endif /* __NXT_TOUCH_SENSOR_HPP__ */
