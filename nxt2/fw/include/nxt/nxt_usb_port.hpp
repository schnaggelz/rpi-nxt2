/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_USB_PORT_HPP__
#define __NXT_USB_PORT_HPP__

#include "nxt_usb_data.hpp"

namespace nxt
{
namespace fw
{
class USBPort
{
  public:
    USBPort() = default;

    bool isConnected() const noexcept;

    bool read(USBData& data) noexcept;
    bool write(USBData& data) noexcept;

    void init() noexcept;
    void exit() noexcept;
};

} // namespace fw
} // namespace nxt

#endif /* __NXT_USB_PORT_HPP__ */
