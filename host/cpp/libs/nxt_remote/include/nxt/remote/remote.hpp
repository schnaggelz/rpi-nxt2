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
    static constexpr std::uint16_t VERSION = 1;

  public:
    using Port = nxt::com::protocol::Port;
    using Info = nxt::com::protocol::Info;

    bool connect();
    bool disconnect();
    bool isConnected();
    bool poll();
    bool shutdown();

    bool motorFwd(const Port port, const std::uint8_t speed);
    bool motorRev(const Port port, const std::uint8_t speed);
    bool motorStop(const Port port);
    bool motorCmd(const Port port, const std::int8_t speed,
                  const std::int32_t count, const std::int32_t tolerance);

    std::int32_t sensorRcv(const Port port, std::uint8_t idx);
    std::int32_t motorRcv(const Port port, std::uint8_t idx);
    std::int32_t systemRcv(std::uint8_t idx);
    std::int32_t getStatus();
    std::uint16_t getVersion() const {return VERSION;};

  private:
    bool send(const nxt::com::protocol::Command command,
              const nxt::com::protocol::generic::Data& data);

    bool receive(nxt::com::protocol::Command& command,
                 nxt::com::protocol::generic::Data& data);

  private:
    nxt::com::usb::Device _nxt_usb_dev;
};
} // namespace remote
} // namespace nxt

#endif /* __NXT_REMOTE_REMOTE_HPP__ */
