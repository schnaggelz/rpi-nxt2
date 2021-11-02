#ifndef __NXT_USB_DATA_HPP__
#define __NXT_USB_DATA_HPP__

#include <cstdint>
#include <type_traits>

namespace nxt
{
enum class USBCommand : std::uint16_t
{
    GENERIC = 0x00,
    GET_DISTANCE = 0x10,
    GET_COLORS = 0x11,
    START = 0xA0,
    STOP = 0xA1,
    MOVE_FORWARD = 0xA2,
    MOVE_BACK = 0xA3,
    TURN_LEFT = 0xA4,
    TURN_RIGHT = 0xA5,
    RUN_FASTER = 0xA6,
    RUN_SLOWER = 0xA7,
    UNDEFINED = 0x0F
};

struct USBData
{
    USBData() = default;

    std::uint16_t id;
    std::uint16_t size;
    std::uint32_t data[8];
};

static_assert(sizeof(USBData) == 36, "Invalid USBData size");

} // namespace nxt

#endif /* __NXT_USB_DATA_HPP__ */