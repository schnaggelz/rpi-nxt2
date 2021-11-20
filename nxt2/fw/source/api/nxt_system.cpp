/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "api/nxt_system.hpp"

#include "drivers/nxt_avr.h"

namespace nxt
{
std::int32_t System::getBatteryVoltage() const
{
    return nxt_avr_get_battery_voltage();
}

void System::shutdown() const
{
    nxt_avr_power_down();
}

void System::update() const
{
    nxt_avr_update();
}
} // namespace nxt
