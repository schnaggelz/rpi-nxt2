/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* NXT C++ driver API
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_USB_DATA_HPP__
#define __NXT_USB_DATA_HPP__

#include "protocol/protocol.hpp"

#include <type_traits>

namespace nxt
{
using USBCommand = nxt::protocol::Command;

using USBData = nxt::protocol::Packet;

static_assert(sizeof(USBData) == 36, "Invalid USBData size");

} // namespace nxt

#endif /* __NXT_USB_DATA_HPP__ */