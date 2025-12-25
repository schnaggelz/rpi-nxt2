#pragma once

#include <rclcpp/node.hpp>

#include <libcamera/camera.h>

class CamNode : public rclcpp::Node
{
public:
  CamNode() : Node("camera_node")
  {
    RCLCPP_INFO(this->get_logger(), "Camera node has been started.");
  }
};
