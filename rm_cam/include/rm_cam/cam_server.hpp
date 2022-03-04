#ifndef RM_CAM__CAM_SERVER_HPP
#define RM_CAM__CAM_SERVER_HPP

#include <thread>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "rm_interfaces/srv/get_camera_info.hpp"
#include "rm_cam/cam_interface.hpp"

namespace rm_cam
{
    using cam_info_manager = camera_info_manager::CameraInfoManager;

    // 摄像头服务节点
    class CamServer
    {
    public:
        explicit CamServer(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<CamInterface> cam_interface);

        std::shared_ptr<cam_info_manager> get_camera_info_manager();

    private:
        void timer_callback();
        void cam_info_timer_callback();
        void camera_info_callback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const rm_interfaces::srv::GetCameraInfo::Request::SharedPtr request,
            rm_interfaces::srv::GetCameraInfo::Response::SharedPtr response);

    private:
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
        rclcpp::Service<rm_interfaces::srv::GetCameraInfo>::SharedPtr camera_info_service_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr cam_info_timer_;
        std::shared_ptr<cam_info_manager> camera_info_manager_;

        // device interface
        std::shared_ptr<CamInterface> cam_interface_;

        // data
        double time_stamp_ms_;
        cv::Mat img_;

        sensor_msgs::msg::CameraInfo cam_info_;

        bool has_camera_info_{false};
        int fps_{60};
        int reopen_cnt_{0};
    };
} // namespace rm_cam

#endif // RM_CAM__CAM_SERVER_HPP
