/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Remote control application. Brick is controlled by a Raspberry Pi.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "remote.hpp"

#include "nxt/com/protocol/generic_packet.hpp"

namespace nxt
{
namespace rc
{
void Remote::init()
{
    _usb_port.init();

    for (auto& motor : _motors)
    {
        motor.init();
    }

    _sensor_1.init();
    _sensor_2.init();
}

void Remote::run()
{
    receive();

    process();

    send();

    display();
}

void Remote::process()
{
    auto command =
        nxt::utils::to_enum<nxt::com::protocol::Command>(_usb_data_rx.type);

    _monitor.setLineValue(3, 1, _usb_data_rx.data[0]);
    _monitor.setLineValue(4, 1, _usb_data_rx.data[1]);
    _monitor.setLineValue(5, 1, _usb_data_rx.data[2]);
    _monitor.setLineValue(6, 1, _usb_data_rx.data[3]);

    switch (command)
    {
    case nxt::com::protocol::Command::MOTOR_FWD:
    case nxt::com::protocol::Command::MOTOR_REV:
    case nxt::com::protocol::Command::MOTOR_STP:
    case nxt::com::protocol::Command::MOTOR_CMD:
    {
        const auto port = static_cast<std::uint8_t>(_usb_data_rx.data[0]);
        if (port >= _motors.size())
            break;

        auto speed = _usb_data_rx.data[1];
        auto count = _usb_data_rx.data[2];

        switch (command)
        {
        case nxt::com::protocol::Command::MOTOR_FWD:
            _motors[port].setSpeed(std::abs(speed));
            break;
        case nxt::com::protocol::Command::MOTOR_REV:
            _motors[port].setSpeed(-std::abs(speed));
            break;
        case nxt::com::protocol::Command::MOTOR_STP:
            _motors[port].setSpeed(0);
            _motors[port].resetTarget();
            break;
        case nxt::com::protocol::Command::MOTOR_CMD:
            _motors[port].setTargetCount(count);
            _motors[port].setSpeed(speed);
            break;
        default:
            break;
        }
        break;
    }
    case nxt::com::protocol::Command::FW_UPDATE:
    {
        nxt::fw::System::update();
        break;
    }
    case nxt::com::protocol::Command::POWER_OFF:
    {
        nxt::fw::System::shutdown();
        break;
    }
    default:
        break;
    }
}

void Remote::send()
{
    // Get and send distance
    _usb_data_tx.type =
        nxt::utils::to_underlying(nxt::com::protocol::Command::FULL_DATA);

    namespace generic_protocol = nxt::com::protocol::generic;

    generic_protocol::setCommonData(_usb_data_tx.data, 0,
                                    nxt::fw::System::getBatteryVoltage());

    generic_protocol::setSensorData(_usb_data_tx.data, 0, 0,
                                    _sensor_1.getDistance());

    generic_protocol::setSensorData(_usb_data_tx.data, 1, 0,
                                    _sensor_2.getBrightness());

    generic_protocol::setMotorData(_usb_data_tx.data, 0, 0,
                                   _motors[0].getCurrentCount());

    generic_protocol::setMotorData(_usb_data_tx.data, 1, 0,
                                   _motors[1].getCurrentCount());

    generic_protocol::setMotorData(_usb_data_tx.data, 2, 0,
                                   _motors[2].getCurrentCount());

    _usb_port.write(_usb_data_tx);
}

void Remote::display()
{
    for (auto idx = 0U; idx < _motors.size(); ++idx)
    {
        _monitor.setLineValue(idx, 0, _motors[idx].getCurrentCount());
        _monitor.setLineValue(idx, 1, _motors[idx].getTargetCount());
    }

    _monitor.setLineValue(3, 0, _sensor_1.getDistance());
    _monitor.setLineValue(4, 0, _sensor_2.getBrightness());
}

void Remote::receive()
{
    _usb_port.read(_usb_data_rx);

    for (auto& motor : _motors)
    {
        motor.read();
    }

    _sensor_1.read();
    _sensor_2.read();
}

void Remote::exit()
{
    _usb_port.exit();

    for (auto& motor : _motors)
    {
        motor.exit();
    }

    _sensor_1.exit();
    _sensor_2.exit();
}

} // namespace rc
} // namespace nxt
