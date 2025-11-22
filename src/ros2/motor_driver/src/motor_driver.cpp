#include "motor_driver/motor_driver.hpp"

namespace nxt2
{
MotorDriver::MotorDriver() : Node("motor_driver") 
{
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&MotorDriver::timerCallback, this));
}

void MotorDriver::initialize()
{
    RCLCPP_INFO(this->get_logger(), "MotorDriver initialized");
}

void MotorDriver::control(int speed, bool direction)
{
    RCLCPP_INFO(this->get_logger(), "Controlling motor: speed=%d, direction=%s", speed, direction ? "forward" : "backward");
}
}  // namespace nxt2