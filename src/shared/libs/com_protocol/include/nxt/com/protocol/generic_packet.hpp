/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Minimal remote control communication protocol.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_COM_PROTOCOL_GENERIC_PACKET_HPP__
#define __NXT_COM_PROTOCOL_GENERIC_PACKET_HPP__

#include "nxt/com/protocol.hpp"

#include <cstdint>
#include <limits>

namespace nxt
{
namespace com
{
namespace protocol
{
namespace generic
{
static constexpr std::uint8_t NUM_VALUES_GENERIC = 2U;
static constexpr std::uint8_t NUM_VALUES_PER_SENSOR_PORT = 1U;
static constexpr std::uint8_t NUM_VALUES_PER_MOTOR_PORT = 1U;

struct Packet : com::protocol::Packet
{
    Packet()
    {
        size = NUM_DATA_ELEMENTS;
    }

    static constexpr std::uint8_t NUM_DATA_ELEMENTS =
        NUM_SENSOR_PORTS * NUM_VALUES_PER_SENSOR_PORT +
        NUM_MOTOR_PORTS * NUM_VALUES_PER_MOTOR_PORT + NUM_VALUES_GENERIC;

    std::int32_t data[NUM_DATA_ELEMENTS] = {0};

    static constexpr void validate()
    {
        static_assert(sizeof(generic::Packet) == NUM_BYTES,
                      "Invalid packet size");
    }

    static constexpr std::uint8_t NUM_BYTES =
        sizeof(data) + sizeof(type) + sizeof(size);
};

using Data = decltype(Packet::data);

inline void setCommonData(generic::Data& data, std::uint8_t idx,
                          std::int32_t value)
{
    if (idx >= NUM_VALUES_GENERIC)
        return;

    data[idx] = value;
}

inline void setSensorData(generic::Data& data, std::uint8_t port,
                          std::uint8_t idx, std::int32_t value)
{
    if (port >= NUM_SENSOR_PORTS)
        return;

    if (idx >= NUM_VALUES_PER_SENSOR_PORT)
        return;

    data[NUM_VALUES_GENERIC + NUM_VALUES_PER_SENSOR_PORT * port + idx] = value;
}

inline void setMotorData(generic::Data& data, std::uint8_t port,
                         std::uint8_t idx, std::int32_t value)
{
    if (port >= NUM_MOTOR_PORTS)
        return;

    if (idx >= NUM_VALUES_PER_MOTOR_PORT)
        return;

    data[NUM_VALUES_GENERIC + NUM_SENSOR_PORTS * NUM_VALUES_PER_SENSOR_PORT +
         NUM_VALUES_PER_MOTOR_PORT * port + idx] = value;
}

inline std::int32_t getCommonData(generic::Data& data, std::uint8_t idx)
{
    if (idx >= NUM_VALUES_GENERIC)
        return std::numeric_limits<std::int32_t>::max();

    return data[idx];
}

inline std::int32_t getSensorData(generic::Data& data, std::uint8_t port,
                                  std::uint8_t idx)
{
    if (port >= NUM_SENSOR_PORTS)
        return std::numeric_limits<std::int32_t>::max();

    if (idx >= NUM_VALUES_PER_SENSOR_PORT)
        return std::numeric_limits<std::int32_t>::max();

    return data[NUM_VALUES_GENERIC + NUM_VALUES_PER_SENSOR_PORT * port + idx];
}

inline std::int32_t getMotorData(generic::Data& data, std::uint8_t port,
                                 std::uint8_t idx)
{
    if (port >= NUM_MOTOR_PORTS)
        return std::numeric_limits<std::int32_t>::max();

    if (idx >= NUM_VALUES_PER_MOTOR_PORT)
        return std::numeric_limits<std::int32_t>::max();

    return data[NUM_VALUES_GENERIC +
                NUM_SENSOR_PORTS * NUM_VALUES_PER_SENSOR_PORT +
                NUM_VALUES_PER_MOTOR_PORT * port + idx];
}

} // namespace generic
} // namespace protocol
} // namespace com
} // namespace nxt

#endif /* __NXT_COM_PROTOCOL_GENERIC_PACKET_HPP__ */