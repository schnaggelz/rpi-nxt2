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

#include <cstdint>

namespace nxt
{
namespace fw
{
namespace system
{
    std::int32_t getBatteryVoltage() noexcept;

    void update() noexcept;
    void shutdown() noexcept;
    void wait(std::uint32_t time_ms);
    void beep(std::uint32_t time_ms);
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_SYSTEM_HPP__ */
