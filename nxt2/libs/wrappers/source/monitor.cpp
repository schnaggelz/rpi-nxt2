/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT monitor wrapper for displaying status messages on the LCD screen.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "wrappers/monitor.hpp"

#include "drivers/nxt_display.h"

namespace nxt
{
namespace wrappers
{
void Monitor::init()
{
    nxt_display_clear(0);
    nxt_display_goto_xy(0, 0);

    if (_title == nullptr)
    {
        nxt_display_string(BIOS_VERSION);
    }
    else
    {
        nxt_display_string(_title);
    }

    nxt_display_update();
}

void Monitor::update()
{
    for (auto i = 0U; i < _values.size(); ++i)
    {
        nxt_display_goto_xy(0, i + 1);
        nxt_display_string(_values[i].name);
        nxt_display_goto_xy(4, i + 1);
        nxt_display_signed(_values[i].value, 6);
    }

    nxt_display_update();
}

void Monitor::setLineName(std::uint8_t line, char* name)
{
    if (line < _values.size())
        _values[line].name = name;
}

void Monitor::setLineValue(std::uint8_t line, std::int32_t value)
{
    if (line < _values.size())
        _values[line].value = value;
}

} // namespace libs
} // namespace nxt
