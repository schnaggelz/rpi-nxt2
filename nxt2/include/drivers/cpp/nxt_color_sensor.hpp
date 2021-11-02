#ifndef __NXT_COLOR_SENSOR_HPP__
#define __NXT_COLOR_SENSOR_HPP__

#include <stdint.h>

#include <array>

#include "../../../source/drivers/cpp/nxt_sensor.hpp"

namespace nxt
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
        ColorSensor(uint8_t port_number)
            : Sensor(port_number)
            , _sensor_mode(ColorSensorMode::LIGHT_SENSOR_NONE)
        {
        }

        void setMode(ColorSensorMode mode);

        int16_t getColor(Colors col);
        int16_t getLight();

        void init() override;
        void read() override;
        void exit() override;

    private:

        uint16_t _light_data;
        std::array<int16_t, 3> _color_data;

        ColorSensorMode _sensor_mode;
    };
};

#endif /* __NXT_COLOR_SENSOR_HPP__ */
