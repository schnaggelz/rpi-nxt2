#include "nxt_drivers/driver_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DriverNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
