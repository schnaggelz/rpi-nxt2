#ifndef __NXT_LIGHT_SENSOR_HPP__
#define __NXT_LIGHT_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
    class LightSensor : public Sensor
    {
    public:
        LightSensor(uint8_t port_number)
            : Sensor(port_number)
            , _brightness(0)
        {
        }

        void init() override;
        void read() override;
        void exit() override;

        int16_t getBrightness() { return _brightness; };

    private:

        int16_t _brightness;
    };
};

#endif /* __NXT_LIGHT_SENSOR_HPP__ */
