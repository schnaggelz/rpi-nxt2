/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "api/nxt_usb_port.hpp"

#include "drivers/nxt_usb.h"

namespace nxt
{
namespace fw
{
void USBPort::init()
{
    nxt_usb_init();
}

void USBPort::exit()
{
    nxt_usb_term();
}

bool USBPort::isConnected() const
{
    return nxt_usb_connected() > 0 ? true : false;
}

bool USBPort::read(USBData& data)
{
    std::int32_t bytes_read =
        nxt_usb_read(reinterpret_cast<std::uint8_t*>(&data), 0, sizeof(data));

    return bytes_read > 0 ? true : false;
}

bool USBPort::write(USBData& data)
{
    std::int32_t bytes_written =
        nxt_usb_write(reinterpret_cast<std::uint8_t*>(&data), 0, sizeof(data));

    return bytes_written > 0 ? true : false;
}

} // namespace fw
} // namespace nxt