/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
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
namespace fw
{
class DistanceSensor : public Sensor
{
  public:
    DistanceSensor(Port port)
        : Sensor(port)
        , _current_distance(0)
    {
    }

    std::int32_t getDistance() const noexcept
    {
        return _current_distance;
    }

    static constexpr std::int16_t MAX_DISTANCE = 200;

    void init() noexcept override;
    void read() noexcept override;
    void exit() noexcept override;

  private:
    std::int32_t _current_distance;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_DISTANCE_SENSOR_HPP__ */
