/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Minimal remote control communication protocol.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_COM_PROTOCOL_HPP__
#define __NXT_COM_PROTOCOL_HPP__

#include <cstdint>

namespace nxt
{
namespace com
{
namespace protocol
{
enum class Command : std::uint8_t
{
    GENERIC_M = 0x00,
    GET_SONAR = 0x10,
    GET_COLOR = 0x11,
    GET_LIGHT = 0x12,
    MOTOR_FWD = 0x20,
    MOTOR_REV = 0x21,
    MOTOR_TGT = 0x22,
    MOTOR_STP = 0x2F,
    UNDEFINED = 0xFF
};

enum class Port : std::uint8_t
{
    PORT_A = 0x00,
    PORT_B = 0x01,
    PORT_C = 0x02,
    PORT_1 = 0x00,
    PORT_2 = 0x01,
    PORT_3 = 0x02,
    PORT_4 = 0x03,
    NONE = 0xFF
};

struct Packet
{
    Packet() = default;

    std::uint16_t command;
    std::uint16_t size;

    static constexpr std::uint8_t NUM_DATA_ELEMENTS = 8U;

    std::int32_t data[NUM_DATA_ELEMENTS];

    static constexpr void validate()
    {
        static_assert(sizeof(Packet) == 2 + 2 + NUM_DATA_ELEMENTS * 4,
                      "Invalid USBData size");
    }
};

using Data = decltype(Packet::data);

} // namespace protocol
} // namespace com
} // namespace nxt

#endif /* __NXT_COM_PROTOCOL_HPP__ */