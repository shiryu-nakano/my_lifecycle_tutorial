#include "my_lifecycle_tutorial/supervisor_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lifecycle_supervisor::SupervisorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
