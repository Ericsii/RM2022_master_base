#pragma once
#ifndef RM_CAM__DAHENG_CAM_NODE_HPP
#define RM_CAM__DAHENG_CAM_NODE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rm_cam/cam_server.hpp"
#include "rm_entity_cam/daheng_cam.hpp"

namespace rm_cam
{
    // Node warpper for DaHengCamera
    class DaHengCamNode
    {
    public:
        explicit DaHengCamNode(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rm_cam::DaHengCam> cam_dev_;
        std::shared_ptr<rm_cam::CamServer> cam_server_;
    };
}

#endif // RM_CAM__DAHENG_CAM_NODE_HPP