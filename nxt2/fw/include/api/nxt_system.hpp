/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_SYSTEM_HPP__
#define __NXT_SYSTEM_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace fw
{
class System
{
  public:
    System() = delete;

    static std::int32_t getBatteryVoltage() noexcept;

    static void update() noexcept;
    static void shutdown() noexcept;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_SYSTEM_HPP__ */
