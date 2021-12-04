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
enum class ColorSensorMode : std::uint8_t
{
    COLOR_SENSOR = 0,
    LIGHT_SENSOR_RED,
    LIGHT_SENSOR_GREEN,
    LIGHT_SENSOR_BLUE,
    LIGHT_SENSOR_NONE
};

enum class BasicColor : std::uint8_t
{
    RED = 0,
    GREEN = 1,
    BLUE = 2,
    NONE
};

enum class DetectedColor : std::uint8_t
{
    BLACK = 0,
    BLUE = 1,
    GREEN = 2,
    YELLOW = 3,
    ORANGE = 4,
    RED = 5,
    WHITE = 6,
    UNKNOWN = 99,
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

    std::int16_t getLight() const noexcept;
    std::int16_t getColor(BasicColor col) const noexcept;

    DetectedColor getColor() const noexcept;

    void init() noexcept override;
    void read() noexcept override;
    void exit() noexcept override;

  private:
    ColorSensorMode _sensor_mode;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_COLOR_SENSOR_HPP__ */
