/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_SENSOR_HPP__
#define __NXT_SENSOR_HPP__

#include "nxt/utils/conversion.hpp"

#include <array>
#include <cstdint>

namespace nxt
{
namespace fw
{
class Sensor
{
  public:
    enum class Port : std::uint8_t
    {
        PORT_1 = 0,
        PORT_2 = 1,
        PORT_3 = 2,
        PORT_4 = 3,
    };

  public:
    Sensor(Port port)
        : _port_number(nxt::utils::to_underlying(port)){};

    virtual void init() noexcept{};
    virtual void read() noexcept{};
    virtual void exit() noexcept{};

  protected:
    std::uint8_t _port_number;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_SENSOR_HPP__ */