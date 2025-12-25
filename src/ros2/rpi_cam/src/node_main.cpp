#include "rpi_cam/cam_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CamNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
