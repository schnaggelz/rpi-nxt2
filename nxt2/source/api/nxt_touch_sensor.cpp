#include "../../../include/drivers/cpp/nxt_touch_sensor.hpp"

#include "nxt_touch_sensor.h"

using namespace nxt;

bool TouchSensor::isPressed()
{
    return nxt_touch_sensor_is_pressed(_port_number) > 0 ? true : false;
}
