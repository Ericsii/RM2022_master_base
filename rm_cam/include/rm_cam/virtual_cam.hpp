#ifndef RM_CAM__VIRTUAL_CAM_HPP
#define RM_CAM__VIRTUAL_CAM_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "rm_cam/cam_interface.hpp"

namespace rm_cam
{

    // 虚拟相机类: 使用图片或者视频作为输出
    class VirtualCam : public CamInterface
    {
    public:
        enum
        {
            IMAGE_MODE,
            VIDEO_MODE
        };

        explicit VirtualCam(int mode, const std::string &path, rclcpp::Node::SharedPtr node);

        bool open() override;
        bool close() override;
        bool is_open() override;
        bool grab_img(cv::Mat &image) override;
        bool grab_img(cv::Mat &image, double &timestamp_ms)
        {
            if (this->grab_img(image))
            {
                auto node_time = node_->now();
                timestamp_ms = 0.1 * node_time.seconds();
                return true;
            }
            return false;
        }

    private:
        int current_mode_;
        std::string path_; // 路径(image/video)
        rclcpp::Node::SharedPtr node_;
        cv::Mat img_;      // only for IMAGE_MODE
        cv::VideoCapture cap_; // only for VIDEO_MODE
        int total_frames_;
        int current_frame_;

        bool is_open_;
    };
} // namespace rm_cam

#endif // RM_CAM__VIRTUAL_CAM_HPP
