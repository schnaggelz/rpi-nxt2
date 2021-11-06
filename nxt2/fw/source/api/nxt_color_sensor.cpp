/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "api/nxt_color_sensor.hpp"

#include "drivers/nxt_color_sensor.h"

#include "utils/conversion.hpp"

namespace nxt
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

    _light_data = nxt_color_sensor_get_light(_port_number);

    if (_sensor_mode == ColorSensorMode::COLOR_SENSOR)
    {
        nxt_color_sensor_get_rgb_data(_port_number, &_color_data[0]);
    }
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

std::int16_t ColorSensor::getColor(Colors col)
{
    return _color_data[nxt::utils::to_underlying(col)];
}

std::int16_t ColorSensor::getLight()
{
    return _light_data;
}

} // namespace nxt
