#ifndef __NXT_DISTANCE_SENSOR_HPP__
#define __NXT_DISTANCE_SENSOR_HPP__

#include <stdint.h>

#include "../../../source/drivers/cpp/nxt_sensor.hpp"

namespace nxt
{
    class DistanceSensor : public Sensor
    {
    public:
        DistanceSensor(uint8_t port_number, int32_t max_distance)
            : Sensor(port_number)
            , _current_distance(0)
            , _max_distance(max_distance)
        {
        }

        int32_t getDistance()
        {
            return _current_distance;
        };

        void init() override;
        void read() override;
        void exit() override;

    private:
    
        int32_t _current_distance;
        int32_t _max_distance;
    };
};

#endif /* __NXT_DISTANCE_SENSOR_HPP__ */
