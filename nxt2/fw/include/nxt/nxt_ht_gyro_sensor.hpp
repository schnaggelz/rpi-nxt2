/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_HT_GYRO_SENSOR_HPP__
#define __NXT_HT_GYRO_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace fw
{
namespace ht
{
enum class GyroSensorMode : std::uint8_t
{
    PREVIEW = 0,
    I2C,
};

class GyroSensor : public Sensor
{
   public:
    GyroSensor(Port port)
        : Sensor(port), _mode(GyroSensorMode::PREVIEW), _angular_velocity(0)
    {
    }

    void setMode(GyroSensorMode mode)
    {
        _mode = mode;
    }

    std::uint16_t getAnglarVelocity() const
    {
        return _angular_velocity;
    };

    void init() noexcept override;
    void read() noexcept override;
    void exit() noexcept override;

   private:
    GyroSensorMode _mode;

    std::uint16_t _angular_velocity;
};

}  // namespace ht
}  // namespace fw
}  // namespace nxt

#endif /* __NXT_HT_GYRO_SENSOR_HPP__ */
