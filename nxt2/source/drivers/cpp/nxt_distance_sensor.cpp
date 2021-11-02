#include "../../../include/drivers/cpp/nxt_distance_sensor.hpp"

#include "nxt_sonar_sensor.h"

using namespace nxt;

void DistanceSensor::init()
{
    nxt_sonar_sensor_init(_port_number);
}

void DistanceSensor::exit()
{
    nxt_sonar_sensor_term(_port_number);
}

void DistanceSensor::read()
{
    _current_distance = nxt_sonar_sensor_get_distance(_port_number);
}

