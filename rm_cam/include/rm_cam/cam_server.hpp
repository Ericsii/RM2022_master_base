#ifndef RM_CAM__CAM_SERVER_HPP
#define RM_CAM__CAM_SERVER_HPP

#include <thread>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "rm_interfaces/srv/camera_info.hpp"
#include "rm_cam/cam_interface.hpp"

namespace rm_cam
{
    // 摄像头服务节点
    class CamServer
    {
    public:
        explicit CamServer(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<CamInterface> cam_interface);

    private:
        void timer_callback();
        void camera_info_callback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const rm_interfaces::srv::CameraInfo::Request::SharedPtr request,
            rm_interfaces::srv::CameraInfo::Response::SharedPtr response);

    private:
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
        rclcpp::Service<rm_interfaces::srv::CameraInfo>::SharedPtr camera_info_service_;
        rclcpp::TimerBase::SharedPtr timer_;

        // device interface
        std::shared_ptr<CamInterface> cam_interface_;

        // data
        cv::Mat img_;

        // 相机参数
        std::vector<double> camera_k_;                                     // 3*3 = 9
        std::vector<double> camera_p_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}; // 3*4 = 12
        std::vector<double> camera_d_;

        bool has_camera_info_{false};
        int fps_{60};
        int reopen_cnt_{0};
    };
} // namespace rm_cam

#endif // RM_CAM__CAM_SERVER_HPP
