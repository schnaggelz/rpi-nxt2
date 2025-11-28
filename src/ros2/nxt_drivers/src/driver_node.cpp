#include "nxt_drivers/driver_node.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

DriverNode::DriverNode() : Node("driver_node"), _remote()
{
    _timer =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                std::bind(&DriverNode::timerCallback, this));

    _sensors_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("nxt/sensor_vector", 10);
}

void DriverNode::timerCallback()
{
    if (!_remote.isConnected())
    {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "NXT connected, polling sensors");

    publishSensorData();
}

void DriverNode::publishSensorData()
{
    using Port = nxt::com::protocol::Port;

    std::array<Port, 4> ports = {Port::PORT_1, Port::PORT_2, Port::PORT_3,
                                 Port::PORT_4};
    std::vector<std::int32_t> values;

    std::uint8_t idx = 0U;
    for (auto p : ports)
    {
        int32_t v = _remote.sensorRcv(p, idx);
        values.push_back(v);
        ++idx;
    }

    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data = std::move(values);

    _sensors_pub->publish(msg);
}
