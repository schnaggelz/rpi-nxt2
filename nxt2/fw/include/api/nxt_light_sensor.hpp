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
class LightSensor : public Sensor
{
  public:
    LightSensor(std::uint8_t port_number)
        : Sensor(port_number), _brightness(0)
    {
    }

    void init() override;
    void read() override;
    void exit() override;

    std::int16_t getBrightness()
    {
        return _brightness;
    }

  private:
    std::int16_t _brightness;
};
} // namespace nxt

#endif /* __NXT_LIGHT_SENSOR_HPP__ */
