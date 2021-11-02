#include "api//nxt_touch_sensor.hpp"

#include "drivers/nxt_touch_sensor.h"

namespace nxt
{
bool TouchSensor::isPressed()
{
    return nxt_touch_sensor_is_pressed(_port_number) > 0 ? true : false;
}
} // namespace nxt
