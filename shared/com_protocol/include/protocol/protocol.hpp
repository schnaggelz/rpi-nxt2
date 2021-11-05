/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Minimal remote control communication protocol.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include <cstdint>

namespace nxt
{
namespace protocol
{
enum class Command : std::uint16_t
{
    GENERIC    = 0x00,
    GET_DIST   = 0x10,
    GET_COLOR  = 0x11,
    MOTOR_FWD  = 0x20,
    MOTOR_REV  = 0x21,
    MOTOR_TGT  = 0x22,
    MOTOR_STOP = 0x2F,
    UNDEFINED  = 0xFF
};

struct Packet
{
    Packet() = default;

    std::uint16_t command;
    std::uint16_t size;
    std::uint32_t data[8];
};

static_assert(sizeof(Packet) == 36, "Invalid USBData size");

} // namespace protocol
} // namespace nxt
