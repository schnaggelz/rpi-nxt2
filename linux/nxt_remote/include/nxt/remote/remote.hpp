/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ communication library
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#ifndef __NXT_REMOTE_REMOTE_HPP__
#define __NXT_REMOTE_REMOTE_HPP__

#include "nxt/usb/device.hpp"

#include "nxt/com/protocol.hpp"

namespace nxt
{
namespace remote
{
class Remote
{
  public:
    using Port = nxt::com::protocol::Port;

    bool connect();
    bool disconnect();

    bool motorFwd(const Port port, const std::uint8_t speed);
    bool motorRev(const Port port, const std::uint8_t speed);

  private:
    bool send(const nxt::com::protocol::Command command,
              const nxt::com::protocol::Data& data);

    bool receive(nxt::com::protocol::Command& command,
                 nxt::com::protocol::Data& data);

  private:
    nxt_com::usb::Device _nxt_usb_dev;
};
} // namespace remote
} // namespace nxt_remote

#endif /* __NXT_REMOTE_REMOTE_HPP__ */
