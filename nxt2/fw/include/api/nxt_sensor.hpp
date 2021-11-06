/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_SENSOR_HPP__
#define __NXT_SENSOR_HPP__

#include "nxt_port_enum.hpp"

#include <array>
#include <cstdint>

namespace nxt
{

class Sensor
{
  public:
    Sensor(std::uint8_t port_number) : _port_number(port_number){};

    virtual void init(){};
    virtual void read(){};
    virtual void exit(){};

  protected:
    std::uint8_t _port_number;
};

} // namespace nxt

#endif /* __NXT_SENSOR_HPP__ */