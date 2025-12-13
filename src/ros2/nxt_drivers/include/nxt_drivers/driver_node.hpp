#pragma once

#include "rclcpp/rclcpp.hpp"

#include "nxt_msgs/msg/motor_command.hpp"
#include "nxt_msgs/msg/simple_motor_command.hpp"
#include "nxt_msgs/msg/sensor_data.hpp"

#include "nxt/remote/remote.hpp"

#include "sensor_msgs/msg/range.hpp"

#include <array>

class DriverNode : public rclcpp::Node
{
   public:
    DriverNode();
    ~DriverNode();

   private:
    using Port = nxt::com::protocol::Port;

    void timer_callback();
    void simple_motor_command_callback(const nxt_msgs::msg::SimpleMotorCommand::SharedPtr msg);
    void motor_command_callback(const nxt_msgs::msg::MotorCommand::SharedPtr msg);
    void publish_sensor_data();

   private:
    rclcpp::TimerBase::SharedPtr _timer;
    
    rclcpp::Publisher<nxt_msgs::msg::SensorData>::SharedPtr _sensor_data_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr _range1_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr _range2_pub;
    rclcpp::Subscription<nxt_msgs::msg::SimpleMotorCommand>::SharedPtr _simple_motor_command_sub;
    rclcpp::Subscription<nxt_msgs::msg::MotorCommand>::SharedPtr _motor_command_sub;
    
    nxt::remote::Remote _remote;

    double _range_field_of_view = 0.1;

    static constexpr std::array<Port, 4> SENSOR_PORTS = {
        Port::PORT_1, Port::PORT_2, Port::PORT_3, Port::PORT_4};

    static constexpr std::array<Port, 3> MOTOR_PORTS = {
        Port::PORT_A, Port::PORT_B, Port::PORT_C};
};
