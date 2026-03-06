#pragma once

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <libcamera/camera.h>

class CamNode : public rclcpp::Node
{
   public:
    CamNode() : Node("camera_node")
    {
        RCLCPP_INFO(this->get_logger(), "Camera node has been started.");
    }

   private:
    libcamera::Camera* _camera{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image_raw;
};
