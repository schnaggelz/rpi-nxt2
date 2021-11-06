/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Remote control application. Brick is controlled by a Raspberry Pi.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_APPS_REMOTE_HPP__
#define __NXT_APPS_REMOTE_HPP__

#include "base/runnable_base.hpp"

#include "utils/conversion.hpp"

#include "api/nxt_usb_port.hpp"
#include "api/nxt_motor.hpp"

#include "wrappers/monitor.hpp"

#include <array>
#include <cstdint>

namespace nxt
{
namespace apps
{
class Remote
{
  public:
    Remote(nxt::wrappers::Monitor& monitor)
        : _monitor(monitor)
        , _motor_A(nxt::Actuator::Port::PORT_A, true)
        , _motor_B(nxt::Actuator::Port::PORT_B, true)
        , _motor_C(nxt::Actuator::Port::PORT_C, true)
    {
    }

    ~Remote() = default;

    void init();
    void run();
    void exit();

  protected:
    static constexpr uint8_t NUM_DATA_BYTES = 8;
    using DataArray = std::array<std::uint32_t, NUM_DATA_BYTES>;

  private:
    void process();
    void send();
    void receive();

  private:
    nxt::wrappers::Monitor& _monitor;

    nxt::Motor _motor_A;
    nxt::Motor _motor_B;
    nxt::Motor _motor_C;

    nxt::USBPort _usb_port;
    nxt::USBData _usb_data_rx;
    nxt::USBData _usb_data_tx;

    DataArray _data;
};
} // namespace libs
} // namespace nxt

#endif /* __NXT_APPS_REMOTE_HPP__ */