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

#include "nxt/utils/conversion.hpp"

#include "api/nxt_light_sensor.hpp"
#include "api/nxt_distance_sensor.hpp"
#include "api/nxt_motor.hpp"
#include "api/nxt_usb_port.hpp"
#include "api/nxt_system.hpp"

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
        , _sensor_1(nxt::Sensor::Port::PORT_1)
        , _sensor_2(nxt::Sensor::Port::PORT_2)
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

    nxt::DistanceSensor _sensor_1;
    nxt::LightSensor _sensor_2;

    nxt::USBPort _usb_port;

    nxt::USBData _usb_data_rx;
    nxt::USBData _usb_data_tx;

    DataArray _data;
};
} // namespace apps
} // namespace nxt

#endif /* __NXT_APPS_REMOTE_HPP__ */