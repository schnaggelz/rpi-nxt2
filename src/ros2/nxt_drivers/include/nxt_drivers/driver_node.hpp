#pragma once

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32_multi_array.hpp"

#include "nxt/remote/remote.hpp"

class DriverNode : public rclcpp::Node
{
   public:
    DriverNode();

   private:
    void timerCallback();
    void publishSensorData();

   private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr _sensors_pub;
    
    nxt::remote::Remote _remote;
};
