/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * NXT status monitor helper for displaying status messages on the screen
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#ifndef __STATUS_MONITOR_HPP__
#define __STATUS_MONITOR_HPP__

#include <cstdint>
#include <array>

#define BIOS_VERSION "NXT BIOS 0.03"

namespace nxt
{
namespace libs
{


class StatusMonitor
{
    static constexpr std::uint8_t NUM_VALUE_LINES = 7;

  public:
    StatusMonitor() : _values(), _title(nullptr) {}

    void setTitle(const char* title)
    {
        _title = title;
    }

    void setLineName(std::uint16_t line, char* name);
    void setLineValue(std::uint16_t line, int32_t value);

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

#endif /* __STATUS_MONITOR_HPP__ */
