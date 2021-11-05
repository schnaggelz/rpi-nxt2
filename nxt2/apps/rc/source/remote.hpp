/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Remote control application. Brick is controlled by a Raspberry Pi.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_APPS_REMOTE_HPP__
#define __NXT_APPS_REMOTE_HPP__

#include <array>
#include <cstdint>

#include "api/nxt_utils.hpp"
#include "api/nxt_usb_port.hpp"

namespace nxt
{
namespace apps
{
class Remote
{
  public:
    void init();
    void run();

  private:
    nxt::USBPort _usb_port;

    std::array<std::uint32_t, 8> _data;
};
} // namespace libs
} // namespace nxt

#endif /* __NXT_APPS_REMOTE_HPP__ */