#ifndef __NXT_SENSOR_HPP__
#define __NXT_SENSOR_HPP__

#include <stdint.h>

#include <array>

#include "nxt_port_enum.hpp"

namespace nxt
{

class Sensor
{
public:
    Sensor(uint8_t port_number) 
        : _port_number(port_number) {};

    virtual void init() {};
    virtual void read() {};
    virtual void exit() {};

protected:

    uint8_t _port_number;
};

} // nxt

#endif /* __NXT_SENSOR_HPP__ */