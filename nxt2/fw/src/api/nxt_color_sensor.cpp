/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "api/nxt_color_sensor.hpp"

#include "drivers/nxt_color_sensor.h"

namespace nxt
{
namespace fw
{
void ColorSensor::init()
{
    nxt_color_sensor_init(_port_number);
}

void ColorSensor::exit()
{
    nxt_color_sensor_term(_port_number);
}

void ColorSensor::read()
{
    nxt_color_sensor_update(_port_number);
}

void ColorSensor::setMode(ColorSensorMode mode)
{
    switch (mode)
    {
    case ColorSensorMode::COLOR_SENSOR:
        nxt_color_sensor_set_mode(_port_number, NXT_COLORSENSOR);
        break;

    case ColorSensorMode::LIGHT_SENSOR_RED:
        nxt_color_sensor_set_mode(_port_number, NXT_LIGHTSENSOR_RED);
        break;

    case ColorSensorMode::LIGHT_SENSOR_GREEN:
        nxt_color_sensor_set_mode(_port_number, NXT_LIGHTSENSOR_GREEN);
        break;

    case ColorSensorMode::LIGHT_SENSOR_BLUE:
        nxt_color_sensor_set_mode(_port_number, NXT_LIGHTSENSOR_BLUE);
        break;

    case ColorSensorMode::LIGHT_SENSOR_NONE:
        nxt_color_sensor_set_mode(_port_number, NXT_LIGHTSENSOR_NONE);
        break;
    }

    _sensor_mode = mode;
}

std::int16_t ColorSensor::getLight() const
{
    return nxt_color_sensor_get_light(_port_number);
}

std::int16_t ColorSensor::getColor(BasicColor col) const
{
    if (_sensor_mode == ColorSensorMode::COLOR_SENSOR)
    {
        sint16 rgb_data[3];
        nxt_color_sensor_get_rgb_data(_port_number, &rgb_data[0]);

        return rgb_data[nxt::utils::to_underlying(col)];
    }

    return 0;
}

DetectedColor ColorSensor::getColor() const noexcept
{
    if (_sensor_mode == ColorSensorMode::COLOR_SENSOR)
    {
        const auto color = nxt_color_sensor_get_color(_port_number);

        switch (color)
        {
        case NXT_COLOR_BLACK:
            return DetectedColor::BLACK;

        case NXT_COLOR_WHITE:
            return DetectedColor::WHITE;

        case NXT_COLOR_RED:
            return DetectedColor::RED;

        case NXT_COLOR_GREEN:
            return DetectedColor::GREEN;

        case NXT_COLOR_BLUE:
            return DetectedColor::BLUE;

        case NXT_COLOR_ORANGE:
            return DetectedColor::ORANGE;

        case NXT_COLOR_YELLOW:
            return DetectedColor::YELLOW;

        default:
            return DetectedColor::UNKNOWN;
        }
    }

    return DetectedColor::NONE;
}

} // namespace fw
} // namespace nxt
