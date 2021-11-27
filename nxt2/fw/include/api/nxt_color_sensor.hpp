/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_COLOR_SENSOR_HPP__
#define __NXT_COLOR_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace fw
{
enum class ColorSensorMode
{
    COLOR_SENSOR,
    LIGHT_SENSOR_RED,
    LIGHT_SENSOR_GREEN,
    LIGHT_SENSOR_BLUE,
    LIGHT_SENSOR_NONE
};

enum class Colors
{
    RED,
    GREEN,
    BLUE,
    NONE
};

class ColorSensor : public Sensor
{
  public:
    ColorSensor(Port port)
        : Sensor(port)
        , _sensor_mode(ColorSensorMode::LIGHT_SENSOR_NONE)
    {
    }

    void setMode(ColorSensorMode mode);

    std::int16_t getColor(Colors col) const;
    std::int16_t getLight() const;

    void init() override;
    void read() override;
    void exit() override;

  private:
    std::uint16_t _light_data;
    std::array<std::int16_t, 3> _color_data;

    ColorSensorMode _sensor_mode;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_COLOR_SENSOR_HPP__ */
