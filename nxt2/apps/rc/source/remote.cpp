/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Remote control application. Brick is controlled by a Raspberry Pi.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "remote.hpp"

namespace nxt
{
namespace apps
{
void Remote::init()
{
    _usb_port.init();

    _motor_A.init();
    _motor_B.init();
    _motor_C.init();

    _sensor_1.init();
    _sensor_2.init();
}

void Remote::run()
{
    receive();

    process();

    send();
}

void Remote::process()
{
    auto command =
        nxt::utils::to_enum<nxt::com::protocol::Command>(_usb_data_rx.type);

    if (command == nxt::com::protocol::Command::MOTOR_FWD ||
        command == nxt::com::protocol::Command::MOTOR_REV ||
        command == nxt::com::protocol::Command::MOTOR_STP)
    {
        auto port_idx =
            nxt::utils::to_enum<nxt::com::protocol::Port>(_usb_data_rx.data[0]);

        auto value = 0;

        if (command == nxt::com::protocol::Command::MOTOR_FWD)
        {
            value = _usb_data_rx.data[1];
        }

        else if (command == nxt::com::protocol::Command::MOTOR_REV)
        {
            value = -_usb_data_rx.data[1];
        }

        switch (port_idx)
        {
        case nxt::com::protocol::Port::PORT_A:
            _monitor.setLineValue(0, value);
            _motor_A.setSpeed(value);
            break;
        case nxt::com::protocol::Port::PORT_B:
            _monitor.setLineValue(1, value);
            _motor_B.setSpeed(value);
            break;
        case nxt::com::protocol::Port::PORT_C:
            _monitor.setLineValue(2, value);
            _motor_C.setSpeed(value);
            break;
        default:
            break;
        }
    }
    else if (command == nxt::com::protocol::Command::FW_UPDATE)
    {
        nxt::System::update();
    }
    else if (command == nxt::com::protocol::Command::POWER_OFF)
    {
        nxt::System::shutdown();
    }
}

void Remote::send()
{
    // Get and send distance
    _usb_data_tx.type =
        nxt::utils::to_underlying(nxt::com::protocol::Command::FULL_DATA);

    _usb_data_tx.data[nxt::com::protocol::BATTERY_VOLTAGE] =
        nxt::System::getBatteryVoltage();

    auto base_idx = nxt::com::protocol::NUM_VALUES_GENERIC;

    _usb_data_tx.data[base_idx] = _sensor_1.getDistance();

    base_idx += nxt::com::protocol::NUM_VALUES_PER_DATA_PORT;

    _usb_data_tx.data[base_idx] = _sensor_2.getBrightness();

    _usb_port.write(_usb_data_tx);

    _monitor.setLineValue(4, _sensor_1.getDistance());
    _monitor.setLineValue(5, _sensor_2.getBrightness());
}

void Remote::receive()
{
    _usb_port.read(_usb_data_rx);

    _sensor_1.read();
    _sensor_2.read();
}

void Remote::exit()
{
    _usb_port.exit();

    _motor_A.exit();
    _motor_B.exit();
    _motor_C.exit();

    _sensor_1.exit();
    _sensor_2.exit();
}

} // namespace apps
} // namespace nxt
