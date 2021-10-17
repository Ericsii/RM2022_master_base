#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rm_cam/virtual_cam_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_cam::VirtualCamNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
