/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_USB_DATA_HPP__
#define __NXT_USB_DATA_HPP__

#include "nxt/com/protocol.hpp"
#include "nxt/com/protocol/generic_packet.hpp"

#include <type_traits>

namespace nxt
{
using USBData = nxt::com::protocol::generic::Packet;

static_assert(sizeof(USBData) == 3 * 1 * 4 + 4 * 1 * 4 + 2 * 4 + 4,
              "Invalid USBData size");
} // namespace nxt

#endif /* __NXT_USB_DATA_HPP__ */