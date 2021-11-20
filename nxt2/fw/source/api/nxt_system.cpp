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
std::int32_t System::getBatteryVoltage()
{
    return nxt_avr_get_battery_voltage();
}

void System::shutdown()
{
    nxt_avr_power_down();
}

void System::update()
{
    nxt_avr_update();
}
} // namespace nxt
