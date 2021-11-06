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
        nxt::utils::to_enum<nxt::protocol::Command>(_usb_data_rx.command);

    if (command == nxt::protocol::Command::MOTOR_FWD ||
        command == nxt::protocol::Command::MOTOR_REV ||
        command == nxt::protocol::Command::MOTOR_STOP)
    {
        auto port = _usb_data_rx.data[0];
        auto value = 0;

        if (command == nxt::protocol::Command::MOTOR_FWD)
        {
            value = _usb_data_rx.data[1];
        }
        else if (command == nxt::protocol::Command::MOTOR_REV)
        {
            value = -_usb_data_rx.data[1];
        }

        switch (port)
        {
        case 1:
            _monitor.setLineValue(0, value);
            _motor_A.setSpeed(value);
            break;
        case 2:
            _monitor.setLineValue(1, value);
            _motor_B.setSpeed(value);
            break;
        case 3:
            _monitor.setLineValue(2, value);
            _motor_C.setSpeed(value);
            break;
        }
    }
}

void Remote::send()
{
    static std::uint8_t counter = 0;

    _usb_data_tx.command =
        nxt::utils::to_underlying(nxt::protocol::Command::GENERIC);

    // Send some test data
    _usb_data_tx.data[0] = counter + 1;
    _usb_data_tx.data[1] = counter + 2;
    _usb_data_tx.data[2] = counter + 3;
    _usb_data_tx.data[3] = counter + 4;
    _usb_data_tx.data[4] = counter + 5;
    _usb_data_tx.data[5] = counter + 6;
    _usb_data_tx.data[6] = counter + 7;
    _usb_data_tx.data[7] = counter + 8;

    _usb_data_tx.size = _data.size();

    _usb_port.write(_usb_data_tx);

    counter++;
}

void Remote::receive()
{
    _usb_port.read(_usb_data_rx);
}

void Remote::exit()
{
    _usb_port.exit();
}

} // namespace apps
} // namespace nxt
