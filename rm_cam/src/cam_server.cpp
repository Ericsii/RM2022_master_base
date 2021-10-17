#include "rm_cam/cam_server.hpp"

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "rm_interfaces/qos_policy.hpp"

namespace rm_cam
{
    constexpr const CamParam kCamParamTypes[] = {
        CamParam::Width,
        CamParam::Height,
        CamParam::AutoExposure,
        CamParam::Exposure,
        CamParam::Brightness,
        CamParam::AutoWhiteBalance,
        CamParam::WhiteBalance,
        CamParam::Gain,
        CamParam::Gamma,
        CamParam::Contrast,
        CamParam::Saturation,
        CamParam::Hue,
        CamParam::Fps,
        CamParam::RGain,
        CamParam::GGain,
        CamParam::BGain
    };

    constexpr const char *kCamParamTypeNames[] = {
        "width",
        "height",
        "auto_exposure",
        "exposure",
        "brightness",
        "auto_white_balance",
        "white_balance",
        "gain",
        "gamma",
        "contrast",
        "saturation",
        "hue",
        "fps",
        "Rgain",
        "Ggain",
        "Bgain"
    };

    CamServer::CamServer(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<CamInterface> cam_interface)
        : node_(node), cam_interface_(cam_interface)
    {
        int data;
        constexpr int param_num = sizeof(kCamParamTypes) / sizeof(CamParam);
        for (int i = 0; i < param_num; ++i)
        {
            if (cam_interface->get_parameter(kCamParamTypes[i], data))
            {
                node_->declare_parameter(kCamParamTypeNames[i], data);
                node_->get_parameter(kCamParamTypeNames[i], data);
                cam_interface_->set_parameter(kCamParamTypes[i], data);
            }
        }

        // 获取相机FPS
        cam_interface_->get_parameter(CamParam::Fps, fps_);

        // 打开摄像头
        if (!cam_interface_->open())
        {
            RCLCPP_FATAL(node_->get_logger(), "Fail to open camera!");
            return;
        }

        // 如果fps值非法，则设置为默认值30
        if (fps_ <= 0)
        {
            fps_ = 30;
            cam_interface_->set_parameter(CamParam::Fps, 30);
        }

        std::string camera_name = "camera";

        // 定义ROS parameter获取相机参数
        node_->declare_parameter("camera_name", camera_name);
        node_->declare_parameter("camera_k", camera_k_);
        node_->declare_parameter("camera_p", camera_p_);
        node_->declare_parameter("camera_d", camera_d_);
        node_->get_parameter("camera_name", camera_name);
        node_->get_parameter("camera_k", camera_k_);
        node_->get_parameter("camera_p", camera_p_);
        node_->get_parameter("camera_d", camera_d_);

        // check parameters
        if (camera_k_.size() != 9)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "The size of camera intrinsic parameter(%ld) != 9", camera_k_.size());
        }
        if (camera_p_.size() != 12)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "The size of camera intrinsic parameter(%ld) != 12", camera_p_.size());
        }

        // 自定义QoS
        node_->declare_parameter("custom_qos", false);
        auto custom_qos = node_->get_parameter("custom_qos").as_bool();

        // create image publisher
        if (custom_qos)
        {
            rclcpp::QoS img_pub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy); // 图像传输QoS配置

            img_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
                camera_name + "/image_raw",
                img_pub_qos_profile);
        }
        else
        {
            img_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
                camera_name + "/image_raw",
                1);
        }

        // publisher timer
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000. / fps_));
        timer_ = node_->create_wall_timer(period_ms, std::bind(&CamServer::timer_callback, this));

        // create CameraInfo service
        using namespace std::placeholders;
        camera_info_service_ = node_->create_service<rm_interfaces::srv::CameraInfo>(
            camera_name + "/get_camera_info",
            std::bind(&CamServer::camera_info_callback, this, _1, _2, _3));

        RCLCPP_INFO(
            node_->get_logger(),
            "Camera server init finished.");
    }

    void CamServer::timer_callback()
    {
        if (cam_interface_->grab_img(img_))
        {
            auto header = std_msgs::msg::Header();
            header.stamp = node_->now();

            // publish image msg
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
                                                             header, "bgr8", img_)
                                                             .toImageMsg();

            img_pub_->publish(*img_msg);
        }
        else
        {
            // try to reopen camera
            using namespace std::chrono_literals;
            if (reopen_cnt_ % fps_ == 0)
            {
                cam_interface_->close();
                std::this_thread::sleep_for(100ms);
                if (cam_interface_->open())
                {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Camera reopen success!");
                }
                else
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Camera reopen failed!");
                }
            }
            reopen_cnt_++;
        }
    }

    void CamServer::camera_info_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const rm_interfaces::srv::CameraInfo::Request::SharedPtr request,
        rm_interfaces::srv::CameraInfo::Response::SharedPtr response)
    {
        (void)request_header;
        (void)request;

        auto &camera_info = response->camera_info;
        int data;
        cam_interface_->get_parameter(rm_cam::CamParam::Height, data);
        camera_info.height = data;
        cam_interface_->get_parameter(rm_cam::CamParam::Width, data);
        camera_info.width = data;
        if (camera_k_.size() == 9 && camera_p_.size() == 12)
        {
            std::copy_n(camera_k_.begin(), 9, camera_info.k.begin());
            std::copy_n(camera_p_.begin(), 12, camera_info.p.begin());
            camera_info.d = camera_d_;
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }
} // namespace rm_cam
