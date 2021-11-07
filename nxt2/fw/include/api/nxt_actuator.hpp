/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_ACTUATOR_HPP__
#define __NXT_ACTUATOR_HPP__

#include "nxt/utils/conversion.hpp"

#include <array>
#include <cstdint>

namespace nxt
{
class Actuator
{
  public:
    enum class Port : std::uint8_t
    {
        PORT_A = 0,
        PORT_B = 1,
        PORT_C = 2,
    };

  public:
    Actuator(Port port)
        : _port_number(nxt::utils::to_underlying(port))
    {
    }

    virtual void init(){};
    virtual void read(){};
    virtual void exit(){};

  protected:
    std::uint8_t _port_number;
};

} // namespace nxt

#endif /* __NXT_ACTUATOR_HPP__ */