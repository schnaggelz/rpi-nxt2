#include "api/nxt_buttons.hpp"

#include "drivers/nxt_buttons.h"

namespace nxt
{
bool Buttons::isPressed(Button button)
{
    uint8_t state = 0;

    switch (button)
    {
    case Button::ENTER:
        state = nxt_enter_button_is_pressed();
        break;

    case Button::ESC:
        state = nxt_escape_button_is_pressed();
        break;

    case Button::LEFT:
        state = nxt_left_button_is_pressed();
        break;

    case Button::RIGHT:
        state = nxt_right_button_is_pressed();
        break;
    }

    return state != 0;
}
} // namespace nxt
