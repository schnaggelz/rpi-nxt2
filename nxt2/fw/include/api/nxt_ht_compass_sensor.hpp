/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_HT_COMPASS_SENSOR_HPP__
#define __NXT_HT_COMPASS_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace fw
{
namespace ht
{
class CompassSensor : public Sensor
{
  public:
    CompassSensor(Port port)
        : Sensor(port)
        , _heading(0xFFFF)
    {
    }

    std::uint16_t getHeading() const
    {
        return _heading;
    }

    void init() override;
    void read() override;
    void exit() override;

    bool calibrate();

  private:
    std::int16_t _heading;
};

} // namespace ht
} // namespace fw
} // namespace nxt

#endif /* __NXT_HT_COMPASS_SENSOR_HPP__ */
