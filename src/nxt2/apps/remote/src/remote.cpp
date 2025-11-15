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

#include <cmath>

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

    for (auto& sensor : _distance_sensors)
    {
        sensor.init();
    }

    for (auto& sensor : _color_sensors)
    {
        sensor.init();

        sensor.setMode(fw::ColorSensorMode::COLOR_SENSOR);
    }
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

        switch (command)
        {
        case nxt::com::protocol::Command::MOTOR_FWD:
        {
            auto speed = std::abs(_usb_data_rx.data[1]);
            _motors[port].setSpeed(speed);
        }

        break;
        case nxt::com::protocol::Command::MOTOR_REV:
        {
            auto speed = -std::abs(_usb_data_rx.data[1]);
            _motors[port].setSpeed(speed);
        }
        break;
        case nxt::com::protocol::Command::MOTOR_CMD:
        {
            auto speed = _usb_data_rx.data[1];
            auto count = _usb_data_rx.data[2];
            auto tolerance = _usb_data_rx.data[3];

            _motors[port].setSpeed(speed);
            _motors[port].setTargetCount(count, tolerance);
        }
        break;
        case nxt::com::protocol::Command::MOTOR_STP:
            _motors[port].stop();
            break;
        default:
            break;
        }
        break;
    }
    case nxt::com::protocol::Command::FW_UPDATE:
    {
        nxt::fw::system::update();
        break;
    }
    case nxt::com::protocol::Command::POWER_OFF:
    {
        nxt::fw::system::shutdown();
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
                                    nxt::fw::system::getBatteryVoltage());

    generic_protocol::setMotorData(_usb_data_tx.data, 0, 0,
                                   _motors[0].getCurrentCount());

    generic_protocol::setMotorData(_usb_data_tx.data, 1, 0,
                                   _motors[1].getCurrentCount());

    generic_protocol::setMotorData(_usb_data_tx.data, 2, 0,
                                   _motors[2].getCurrentCount());

    generic_protocol::setSensorData(_usb_data_tx.data, 0, 0,
                                    _distance_sensors[0].getDistance());

    generic_protocol::setSensorData(_usb_data_tx.data, 1, 0,
                                    _distance_sensors[1].getDistance());

    generic_protocol::setSensorData(
        _usb_data_tx.data, 2, 0,
        nxt::utils::to_underlying(_color_sensors[0].getColor()));

    generic_protocol::setSensorData(
        _usb_data_tx.data, 3, 0,
        nxt::utils::to_underlying(_color_sensors[1].getColor()));

    _usb_port.write(_usb_data_tx);
}

void Remote::display()
{
    _monitor.setLineValue(0, 0, _motors[0].getCurrentCount());
    _monitor.setLineValue(0, 1, _motors[0].getTargetCount());

    _monitor.setLineValue(1, 0, _motors[1].getCurrentCount());
    _monitor.setLineValue(1, 1, _motors[1].getTargetCount());

    _monitor.setLineValue(2, 0, _motors[2].getCurrentCount());
    _monitor.setLineValue(2, 1, _motors[2].getTargetCount());

    _monitor.setLineValue(3, 0, _distance_sensors[0].getDistance());
    _monitor.setLineValue(4, 0, _distance_sensors[1].getDistance());

    _monitor.setLineValue(5, 0, _color_sensors[0].getLight());
    _monitor.setLineValue(6, 0, _color_sensors[1].getLight());
}

void Remote::receive()
{
    const auto success = _usb_port.read(_usb_data_rx);

    if (success)
    {
        _monitor.setLineValue(3, 1, _usb_data_rx.data[0]);
        _monitor.setLineValue(4, 1, _usb_data_rx.data[1]);
        _monitor.setLineValue(5, 1, _usb_data_rx.data[2]);
        _monitor.setLineValue(6, 1, _usb_data_rx.data[3]);
    }

    for (auto& motor : _motors)
    {
        motor.read();
    }

    for (auto& sensor : _distance_sensors)
    {
        sensor.read();
    }

    for (auto& sensor : _color_sensors)
    {
        sensor.read();
    }
}

void Remote::exit()
{
    _usb_port.exit();

    for (auto& motor : _motors)
    {
        motor.exit();
    }

    for (auto& sensor : _distance_sensors)
    {
        sensor.exit();
    }

    for (auto& sensor : _color_sensors)
    {
        sensor.exit();
    }
}

} // namespace rc
} // namespace nxt
