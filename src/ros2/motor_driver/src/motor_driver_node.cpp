#include "rclcpp/rclcpp.hpp"
#include "motor_driver/motor_driver.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}