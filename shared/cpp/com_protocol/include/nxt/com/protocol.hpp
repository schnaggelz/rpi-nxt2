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
static constexpr std::uint8_t NUM_SENSOR_PORTS = 4U;
static constexpr std::uint8_t NUM_MOTOR_PORTS = 3U;

namespace com
{
static constexpr std::uint8_t UNDEFINED_TYPE = 0xFF;

namespace protocol
{
enum class Command : std::uint8_t
{
    FULL_DATA = 0x00,
    GET_SONAR = 0x10,
    GET_COLOR = 0x11,
    GET_LIGHT = 0x12,
    MOTOR_FWD = 0x20,
    MOTOR_REV = 0x21,
    MOTOR_CMD = 0x22,
    MOTOR_STP = 0x2F,
    FW_UPDATE = 0xF0,
    POWER_OFF = 0xFA,
    UNDEFINED = UNDEFINED_TYPE
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
    NONE = UNDEFINED_TYPE
};

enum Info : std::uint8_t
{
    BATTERY_VOLTAGE = 0
};

struct Packet
{
    Packet() = default;

    std::uint16_t type {UNDEFINED_TYPE};
    std::uint16_t size {0};
};

} // namespace protocol
} // namespace com
} // namespace nxt

#endif /* __NXT_COM_PROTOCOL_HPP__ */