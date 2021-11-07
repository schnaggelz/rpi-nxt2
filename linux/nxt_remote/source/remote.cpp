/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT remote control library.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "nxt/remote/remote.hpp"

#include "nxt/utils/conversion.hpp"

#include "nxt/usb/device.hpp"

namespace nxt
{
namespace remote
{
nxt::com::protocol::Data _data = {};

bool Remote::connect()
{
    if (_nxt_usb_dev.init())
    {
        if (_nxt_usb_dev.open())
        {
            return true;
        }
        else
        {
            _nxt_usb_dev.exit();
        }
    }

    return false;
}

bool Remote::disconnect()
{
    _nxt_usb_dev.close();

    return true;
}

bool Remote::isConnected()
{
    return _nxt_usb_dev.isReady();
}

bool Remote::poll()
{
    nxt::com::protocol::Command command;

    if (receive(command, _data))
    {
        if (command == nxt::com::protocol::Command::GENERIC_M)
        {
            return true;
        }
    }

    return false;
}

bool Remote::send(const nxt::com::protocol::Command command,
                  const nxt::com::protocol::Data& data)
{
    nxt::com::protocol::Packet packet;

    if (_nxt_usb_dev.isReady())
    {
        packet.command = nxt::utils::to_underlying(command);
        packet.size = 1;

        for (auto idx = 0U; idx < nxt::com::protocol::Packet::NUM_DATA_ELEMENTS;
             ++idx)
        {
            packet.data[idx] = data[idx];
        }

        _nxt_usb_dev.write(packet);

        return true;
    }

    return false;
}

bool Remote::receive(nxt::com::protocol::Command& command,
                     nxt::com::protocol::Data& data)
{
    nxt::com::protocol::Packet packet;

    if (_nxt_usb_dev.isReady())
    {
        _nxt_usb_dev.read(packet);

        for (auto idx = 0U; idx < nxt::com::protocol::Packet::NUM_DATA_ELEMENTS;
             ++idx)
        {
            data[idx] = packet.data[idx];
        }

        command =
            nxt::utils::to_enum<nxt::com::protocol::Command>(packet.command);

        return true;
    }

    return false;
}

bool Remote::motorFwd(const Port port, const std::uint8_t speed)
{
    return send(nxt::com::protocol::Command::MOTOR_FWD,
                {nxt::utils::to_underlying(port), speed});
}

bool Remote::motorRev(const Port port, const std::uint8_t speed)
{
    return send(nxt::com::protocol::Command::MOTOR_REV,
                {nxt::utils::to_underlying(port), speed});
}

bool Remote::motorStop(const Port port)
{
    return send(nxt::com::protocol::Command::MOTOR_STP,
                {nxt::utils::to_underlying(port)});
}

std::int32_t Remote::sensorRcv(const Remote::Port port)
{
    return _data[nxt::utils::to_underlying(port)];
}

} // namespace remote
} // namespace nxt