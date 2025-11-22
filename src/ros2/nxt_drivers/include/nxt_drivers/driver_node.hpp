#pragma once

#include "rclcpp/rclcpp.hpp"

class DriverNode : public rclcpp::Node
{
   public:
    DriverNode();

   private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};
