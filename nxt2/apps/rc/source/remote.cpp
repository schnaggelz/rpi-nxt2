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

    switch (command)
    {
    case nxt::protocol::Command::MOTOR_FWD:
        _monitor.setLineValue(0, _usb_data_rx.data[1]);
        break;
    case nxt::protocol::Command::MOTOR_REV:
        _monitor.setLineValue(0, -_usb_data_rx.data[1]);
        break;
    case nxt::protocol::Command::MOTOR_STOP:
        _monitor.setLineValue(0, 0);
        break;
    default:
        break;
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
