/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_DISTANCE_SENSOR_HPP__
#define __NXT_DISTANCE_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
class DistanceSensor : public Sensor
{
  public:
    DistanceSensor(std::uint8_t port_number, std::int32_t max_distance)
        : Sensor(port_number), _current_distance(0), _max_distance(max_distance)
    {
    }

    std::int32_t getDistance()
    {
        return _current_distance;
    }

    void init() override;
    void read() override;
    void exit() override;

  private:
    std::int32_t _current_distance;
    std::int32_t _max_distance;
};
}; // namespace nxt

#endif /* __NXT_DISTANCE_SENSOR_HPP__ */
