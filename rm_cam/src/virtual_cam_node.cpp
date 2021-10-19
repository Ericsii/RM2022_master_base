#include "rm_cam/virtual_cam_node.hpp"

#include <memory>
#include <string>

namespace rm_cam
{
    VirtualCamNode::VirtualCamNode(
        const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("virtual_cam", options);

        node_->declare_parameter("image_path", "");
        node_->declare_parameter("video_path", "");
        auto image_path = node_->get_parameter("image_path").as_string();
        auto video_path = node_->get_parameter("video_path").as_string();

        // create device
        if (!image_path.empty())
        {
            cam_dev_ = std::make_shared<VirtualCam>(VirtualCam::IMAGE_MODE, image_path);
        }
        else if (!video_path.empty())
        {
            cam_dev_ = std::make_shared<VirtualCam>(VirtualCam::VIDEO_MODE, video_path);
        }
        else
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "image_path or video_path cannot be none.");
            return;
        }

        // create server
        cam_server_ = std::make_shared<CamServer>(node_, cam_dev_);
    }
} // namespace rm_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_cam::VirtualCamNode)
