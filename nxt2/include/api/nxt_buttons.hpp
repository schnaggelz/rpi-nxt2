#ifndef __NXT_BUTTONS_HPP__
#define __NXT_BUTTONS_HPP__

#include <stdint.h>

namespace nxt
{
    enum class Button
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
};

#endif /* __NXT_BUTTONS_HPP__ */