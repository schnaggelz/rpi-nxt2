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

#include "api/nxt_color_sensor.hpp"
#include "api/nxt_distance_sensor.hpp"
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
        , _motors({nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_A),
                   nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_B),
                   nxt::fw::Motor(nxt::fw::Actuator::Port::PORT_C)})
        , _distance_sensors(
              {nxt::fw::DistanceSensor(nxt::fw::Sensor::Port::PORT_1),
               nxt::fw::DistanceSensor(nxt::fw::Sensor::Port::PORT_2)})
        , _color_sensors({nxt::fw::ColorSensor(nxt::fw::Sensor::Port::PORT_3),
                          nxt::fw::ColorSensor(nxt::fw::Sensor::Port::PORT_4)})
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
    std::array<nxt::fw::DistanceSensor, 2> _distance_sensors;
    std::array<nxt::fw::ColorSensor, 2> _color_sensors;

    nxt::fw::USBPort _usb_port;

    nxt::fw::USBData _usb_data_rx;
    nxt::fw::USBData _usb_data_tx;
};
} // namespace rc
} // namespace nxt

#endif /* __NXT_RC_REMOTE_HPP__ */