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
class GyroSensor : public Sensor
{
  public:
    GyroSensor(Port port)
        : Sensor(port)
        , _angular_velocity(0)
    {
    }

    std::uint16_t getAnglarVelocity(bool preview = true) const noexcept;

    void init() noexcept override;
    void read() noexcept override;
    void exit() noexcept override;

  private:
    std::uint16_t _angular_velocity;
    std::uint16_t _angular_velocity_preview;
};

} // namespace ht
} // namespace fw
} // namespace nxt

#endif /* __NXT_HT_GYRO_SENSOR_HPP__ */
