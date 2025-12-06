#include "nxt_drivers/driver_node.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

DriverNode::DriverNode() : Node("driver_node"), _remote()
{
    _timer =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&DriverNode::timer_callback, this));

    _sensor_data_pub = this->create_publisher<nxt_msgs::msg::SensorData>("nxt/sensor_data", 50);

    // subscribe to motor commands
    _simple_motor_command_sub = this->create_subscription<nxt_msgs::msg::SimpleMotorCommand>(
        "nxt/simple_motor_cmd", 10,
        std::bind(&DriverNode::simple_motor_command_callback, this, std::placeholders::_1));

    _motor_command_sub = this->create_subscription<nxt_msgs::msg::MotorCommand>(
        "nxt/motor_cmd", 10,
        std::bind(&DriverNode::motor_command_callback, this, std::placeholders::_1));

    if (_remote.connect())
    {
        RCLCPP_INFO(this->get_logger(), "Connected to NXT");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to NXT, check connection");
    }
}

void DriverNode::simple_motor_command_callback(const nxt_msgs::msg::SimpleMotorCommand::SharedPtr msg)
{
    if (!_remote.isConnected())
    {
        return;
    }

    const auto port_idx = msg->port;
    if (port_idx >= MOTOR_PORTS.size())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid motor port index: %u", port_idx);
        return;
    }

    const auto port = MOTOR_PORTS[port_idx];
    const auto speed = msg->speed;
}

void DriverNode::motor_command_callback(const nxt_msgs::msg::MotorCommand::SharedPtr msg)
{
    if (!_remote.isConnected())
    {
        return;
    }

    const auto port_idx = msg->port;
    if (port_idx >= MOTOR_PORTS.size())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid motor port index: %u", port_idx);
        return;
    }

    const auto port = MOTOR_PORTS[port_idx];
}

void DriverNode::timer_callback()
{
    if (!_remote.isConnected())
    {
        return;
    }

    publish_sensor_data();

    // publish system state
}

void DriverNode::publish_sensor_data()
{
    _remote.poll();

    std::uint8_t port_idx = 0U;
    for (auto port : SENSOR_PORTS)
    {
        auto value = _remote.sensorRcv(port, 0);
        auto msg = nxt_msgs::msg::SensorData();

        msg.port = port_idx;
        msg.data.push_back(value);

        _sensor_data_pub->publish(msg);

        ++port_idx;
    }
}
