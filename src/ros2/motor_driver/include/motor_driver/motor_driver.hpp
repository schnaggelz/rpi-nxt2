#include "rclcpp/rclcpp.hpp"

namespace nxt2
{
class MotorDriver : public rclcpp::Node
{
   public:
    MotorDriver();
    void initialize();
    void control(int speed, bool direction);

   private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace nxt2