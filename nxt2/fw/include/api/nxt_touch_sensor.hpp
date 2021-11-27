/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_TOUCH_SENSOR_HPP__
#define __NXT_TOUCH_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace fw
{
class TouchSensor : public Sensor
{
  public:
    TouchSensor(Port port)
        : Sensor(port)
    {
    }

    bool isPressed() const;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_TOUCH_SENSOR_HPP__ */
