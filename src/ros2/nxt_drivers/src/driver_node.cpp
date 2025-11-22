#include "nxt_drivers/driver_node.hpp"

DriverNode::DriverNode() : Node("driver_node") 
{
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&DriverNode::timerCallback, this));
}

