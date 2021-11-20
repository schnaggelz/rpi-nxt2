/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_DISTANCE_SENSOR_HPP__
#define __NXT_DISTANCE_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
class System
{
  public:
    System() = delete;

    static std::int32_t getBatteryVoltage();

    static void update();
    static void shutdown();
};
} // namespace nxt

#endif /* __NXT_DISTANCE_SENSOR_HPP__ */
