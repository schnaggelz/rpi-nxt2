/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_BUTTONS_HPP__
#define __NXT_BUTTONS_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace fw
{
class Buttons
{
  public:
    enum class Button : std::uint8_t
    {
        ENTER,
        ESC,
        LEFT,
        RIGHT
    };

  public:
    Buttons() = default;

    bool isPressed(Button button) const;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_BUTTONS_HPP__ */