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
    DataArray data;

    // Send some test data
    data[0] = 1;
    data[1] = 2;
    data[2] = 3;
    data[3] = 4;
    data[4] = 5;
    data[5] = 6;
    data[6] = 7;
    data[7] = 8;

    send(data);
}

void Remote::send(const DataArray& data)
{
    _usb_data_tx.command = nxt::to_underlying(nxt::protocol::Command::GENERIC);

    _usb_data_tx.size = data.size();

    for (unsigned i = 0; i < data.size(); ++i)
    {
        _usb_data_tx.data[i] = data[i];
    }

    _usb_port.write(_usb_data_tx);
}

void Remote::exit()
{
    _usb_port.exit();
}

} // namespace apps
} // namespace nxt
