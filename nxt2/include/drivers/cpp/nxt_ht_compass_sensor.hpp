#ifndef __NXT_HT_COMPASS_SENSOR_HPP__
#define __NXT_HT_COMPASS_SENSOR_HPP__

#include <stdint.h>

#include "../../../source/drivers/cpp/nxt_sensor.hpp"

namespace nxt
{
namespace ht
{
    class CompassSensor : public Sensor
    {
    public:
        CompassSensor(uint8_t port_number)
            : Sensor(port_number)
            , _heading(0xFFFF)
        {
        }

        uint16_t getHeading() { return _heading; }

        void init() override;
        void read() override;
        void exit() override;

        bool calibrate();

    private:

        int16_t _heading;

    };
} // ht
} // nxt

#endif /* __NXT_HT_COMPASS_SENSOR_HPP__ */
