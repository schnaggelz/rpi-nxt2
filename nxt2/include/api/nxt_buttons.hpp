#ifndef __NXT_BUTTONS_HPP__
#define __NXT_BUTTONS_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
enum class Button : std::uint8_t
{
    ENTER,
    ESC,
    LEFT,
    RIGHT
};

class Buttons
{
  public:
    Buttons() = default;

    bool isPressed(Button button);
};
} // namespace nxt

#endif /* __NXT_BUTTONS_HPP__ */