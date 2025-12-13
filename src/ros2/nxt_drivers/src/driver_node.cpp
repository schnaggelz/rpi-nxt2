#include "nxt_drivers/driver_node.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>
#include <algorithm>

DriverNode::DriverNode() : Node("driver_node"), _remote()
{
    _timer =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&DriverNode::timer_callback, this));

    _sensor_data_pub = this->create_publisher<nxt_msgs::msg::SensorData>("nxt/sensor_data", 50);
    _range1_pub = this->create_publisher<sensor_msgs::msg::Range>("nxt/range/port1", 10);
    _range2_pub = this->create_publisher<sensor_msgs::msg::Range>("nxt/range/port2", 10);

    // parameter: field of view for range sensors (radians)
    this->declare_parameter<double>("range_field_of_view", _range_field_of_view);
    _range_field_of_view = this->get_parameter("range_field_of_view").as_double();

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

DriverNode::~DriverNode()
{
    if (_remote.isConnected())
    {
        _remote.motorStop(Port::PORT_A);
        _remote.motorStop(Port::PORT_B);
        _remote.motorStop(Port::PORT_C);

        _remote.disconnect();
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
    const auto command = msg->command;
    const auto speed = msg->speed;

    if (command == nxt_msgs::msg::SimpleMotorCommand::STOP)
    {
        RCLCPP_INFO(this->get_logger(), "Stopping motor on port %u", port_idx);
        _remote.motorStop(port);
    }
    else if (command == nxt_msgs::msg::SimpleMotorCommand::FORWARD)
    {
        RCLCPP_INFO(this->get_logger(), "Setting motor on port %u to speed %d (forward)", port_idx, speed);
        _remote.motorFwd(port, speed);
    }
    else if (command == nxt_msgs::msg::SimpleMotorCommand::REVERSE)
    {
        RCLCPP_INFO(this->get_logger(), "Setting motor on port %u to speed %d (reverse)", port_idx, speed);
        _remote.motorRev(port, speed);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid motor command: %u", command);
        return;
    }
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
    const auto count = msg->count;
    const auto speed = msg->speed;
    const auto tolerance = msg->tolerance;

    RCLCPP_INFO(this->get_logger(),
                "Setting motor on port %u to speed %d for %d increments with tolerance %d",
                port_idx, speed, count, tolerance);
    _remote.motorCmd(port, speed, count, tolerance);
}

void DriverNode::timer_callback()
{
    if (!_remote.isConnected())
    {
        return;
    }

    publish_sensor_data();
    publish_system_state();
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

    sensor_msgs::msg::Range rmsg;
    rmsg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    rmsg.field_of_view = static_cast<float>(_range_field_of_view);
    rmsg.min_range = 0.02f;
    rmsg.max_range = 2.0f;

    // port 1
    auto v1 = _remote.sensorRcv(Port::PORT_1, 0);
    rmsg.header.stamp = this->now();
    rmsg.header.frame_id = "nxt_port_1";
    rmsg.range = std::clamp(static_cast<float>(v1) * 0.01f, rmsg.min_range, rmsg.max_range);
    _range1_pub->publish(rmsg);

    // port 2
    auto v2 = _remote.sensorRcv(Port::PORT_2, 0);
    rmsg.header.stamp = this->now();
    rmsg.header.frame_id = "nxt_port_2";
    rmsg.range = std::clamp(static_cast<float>(v2) * 0.01f, rmsg.min_range, rmsg.max_range);
    _range2_pub->publish(rmsg);
}

void DriverNode::publish_system_state()
{
    // Placeholder for future system state publishing
}
