/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT monitor wrapper for displaying status messages on the LCD screen.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#ifndef __NXT_APP_UTILS_MONITOR_HPP__
#define __NXT_APP_UTILS_MONITOR_HPP__

#include <array>
#include <cstdint>

#define BIOS_VERSION "NXT BIOS 0.03"

namespace nxt
{
namespace app_utils
{
class StatusMonitor
{
  public:
    static constexpr std::uint8_t NUM_LINES = 7;
    static constexpr std::uint8_t NUM_VALUES_PER_LINE = 2;
    static constexpr std::uint8_t NUM_CHARS_PER_VALUE = 6;

  public:
    StatusMonitor()
        : _lines()
        , _title(nullptr)
    {
    }

    void setTitle(const char* title)
    {
        _title = title;
    }

    void setLineName(std::uint8_t line, char* name);
    void setLineValue(std::uint8_t line, std::uint8_t idx, std::int32_t value);

    void init();
    void update();

  private:
    struct Line
    {
        char* name = nullptr;

        using Values = std::array<std::int32_t, NUM_VALUES_PER_LINE>;
        Values values = {0, 0};
    };

    std::array<Line, NUM_LINES> _lines;

    const char* _title;
};

} // namespace app_utils
} // namespace nxt

#endif /* __NXT_APP_UTILS_MONITOR_HPP__ */
