/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_LIGHT_SENSOR_HPP__
#define __NXT_LIGHT_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace fw
{
class LightSensor : public Sensor
{
  public:
    LightSensor(Port port)
        : Sensor(port)
        , _brightness(0)
    {
    }

    void init() noexcept override;
    void read() noexcept override;
    void exit() noexcept override;

    std::int16_t getBrightness() const noexcept
    {
        return _brightness;
    }

  private:
    std::int16_t _brightness;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_LIGHT_SENSOR_HPP__ */
