/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * NXT monitor wrapper for displaying status messages on the LCD screen.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#ifndef __NXT_STATMON_MONITOR_HPP__
#define __NXT_STATMON_MONITOR_HPP__

#include <array>
#include <cstdint>

#define BIOS_VERSION "NXT BIOS 0.03"

namespace nxt
{
namespace wrappers
{
class Monitor
{
    static constexpr std::uint8_t NUM_VALUE_LINES = 7;

  public:
    Monitor() : _values(), _title(nullptr) {}

    void setTitle(const char* title)
    {
        _title = title;
    }

    void setLineName(std::uint8_t line, char* name);
    void setLineValue(std::uint8_t line, std::int32_t value);

    void init();
    void update();

  private:
    struct Value
    {
        char* name = nullptr;
        std::int32_t value = 0;
    };

    std::array<Value, NUM_VALUE_LINES> _values;

    const char* _title;
};

} // namespace libs
} // namespace nxt

#endif /* __NXT_STATMON_MONITOR_HPP__ */
