#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rm_base/simple_robot_base_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_base::SimpleRobotBaseNode>();
    // rclcpp::spin(std::make_shared<SimpleRobotBaseNode>());
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
