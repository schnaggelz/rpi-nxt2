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
    for (auto i = 0U; i < _lines.size(); ++i)
    {
        nxt_display_goto_xy(0, i + 1);
        nxt_display_string(_lines[i].name);
        nxt_display_goto_xy(4, i + 1);

        const auto& values = _lines[i].values;

        for (auto j = 0U; j < values.size(); ++j)
        {
            nxt_display_signed(values[j], NUM_CHARS_PER_VALUE);
        }
    }

    nxt_display_update();
}

void Monitor::setLineName(std::uint8_t line, char* name)
{
    if (line < _lines.size())
    {
        _lines[line].name = name;
    }
}

void Monitor::setLineValue(std::uint8_t line, std::uint8_t idx,
                           std::int32_t value)
{
    if (line < _lines.size() && idx < _lines[line].values.size())
    {
        _lines[line].values[idx] = value;
    }
}

} // namespace wrappers
} // namespace nxt
