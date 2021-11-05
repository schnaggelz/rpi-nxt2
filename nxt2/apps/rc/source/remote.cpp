/*******************************************************************************
 * Copyright (C) 2015 T. Reich
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
}

void Remote::run()
{
    // Send some test data
    _data[0] = 1;
    _data[1] = 2;
    _data[2] = 3;
    _data[3] = 4;
    _data[4] = 5;
    _data[5] = 6;
    _data[6] = 7;
    _data[7] = 8;
}

void Remote::send()
{
    _usb_data_tx.id = nxt::to_underlying(
        USBCommand::GENERIC);

    _usb_data_tx.size = _data.size();

    for (unsigned i = 0; i < _data.size(); ++i)
    {
        _usb_data_tx.data[i] = _data[i];
    }

    _usb_port.write(_usb_data_tx);


void Remote::exit()
{
    _usb_port.exit();
}

} // namespace apps
} // namespace nxt
