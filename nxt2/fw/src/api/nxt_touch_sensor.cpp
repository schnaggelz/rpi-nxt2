/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "api//nxt_touch_sensor.hpp"

#include "drivers/nxt_touch_sensor.h"

namespace nxt
{
namespace fw
{
bool TouchSensor::isPressed() const
{
    return nxt_touch_sensor_is_pressed(_port_number) > 0 ? true : false;
}

} // namespace fw
} // namespace nxt
