/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Remote control application. Brick is controlled by a Raspberry Pi.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_RC_REMOTE_HPP__
#define __NXT_RC_REMOTE_HPP__

#include "base/runnable_base.hpp"

#include "nxt/utils/conversion.hpp"

#include "api/nxt_distance_sensor.hpp"
#include "api/nxt_light_sensor.hpp"
#include "api/nxt_motor.hpp"
#include "api/nxt_system.hpp"
#include "api/nxt_usb_port.hpp"

#include "wrappers/monitor.hpp"

#include <array>
#include <cstdint>

namespace nxt
{
namespace rc
{
class Remote
{
  public:
    Remote(nxt::wrappers::Monitor& monitor)
        : _monitor(monitor)
        , _motors({nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_A, true),
                   nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_B, true),
                   nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_C, true)})
        , _sensor_1(nxt::fw::Sensor::Port::PORT_1)
        , _sensor_2(nxt::fw::Sensor::Port::PORT_2)
    {
    }

    ~Remote() = default;

    void init();
    void run();
    void exit();

  private:
    void process();
    void send();
    void receive();
    void display();

  private:
    nxt::wrappers::Monitor& _monitor;

    std::array<nxt::fw::Motor, 3> _motors;

    nxt::fw::DistanceSensor _sensor_1;
    nxt::fw::LightSensor _sensor_2;

    nxt::fw::USBPort _usb_port;

    nxt::fw::USBData _usb_data_rx;
    nxt::fw::USBData _usb_data_tx;
};
} // namespace apps
} // namespace nxt

#endif /* __NXT_RC_REMOTE_HPP__ */