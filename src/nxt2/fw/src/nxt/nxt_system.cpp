/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "nxt/nxt_system.hpp"

#include "drivers/nxt_avr.h"
#include "drivers/nxt_sound.h"

#include "platform/systick.h"

namespace nxt
{
namespace fw
{
namespace system
{
std::int32_t getBatteryVoltage()
{
    return nxt_avr_get_battery_voltage();
}

void shutdown()
{
    nxt_avr_power_down();
}

void update()
{
    nxt_avr_firmware_update();
}

void wait(std::uint32_t time_ms)
{
    systick_wait_ms(time_ms);
}

void beep(std::uint32_t time_ms)
{
    nxt_sound_freq(1000, time_ms);
}

} // namespace system
} // namespace fw
} // namespace nxt
