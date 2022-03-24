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
        CamParam::BGain};

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
        "Bgain"};

    CamServer::CamServer(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<CamInterface> cam_interface)
        : node_(node), cam_interface_(cam_interface)
    {
        std::string camera_name = "camera";
        std::string camera_info_url = "";

        // 定义ROS parameter获取相机参数
        node_->declare_parameter("camera_name", camera_name);
        node_->declare_parameter("best_effort_qos", false); 
        node_->declare_parameter("camera_info_url", camera_info_url);
        node_->get_parameter("camera_name", camera_name);
        node_->get_parameter("camera_info_url", camera_info_url);

        camera_info_manager_ = std::make_shared<cam_info_manager>(
            node_.get(), camera_name, camera_info_url
        );
        if (camera_info_manager_->loadCameraInfo(camera_info_url))
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Load camera info from url: '%s'", camera_info_url.c_str()
            );
        }
        else {
            RCLCPP_WARN(
                node_->get_logger(),
                "Camera info url '%s' is not found!", camera_info_url.c_str()
            );
        }

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

        // 自定义QoS
        auto custom_qos = node_->get_parameter("best_effort_qos").as_bool();

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

        info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            camera_name + "/camera_info",
            5
        );

        // publisher timer
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000. / fps_));
        timer_ = node_->create_wall_timer(period_ms, std::bind(&CamServer::timer_callback, this));
        period_ms = std::chrono::milliseconds(1000);
        cam_info_timer_ = node_->create_wall_timer(period_ms, std::bind(&CamServer::cam_info_timer_callback, this));

        // create CameraInfo service
        using namespace std::placeholders;
        camera_info_service_ = node_->create_service<rm_interfaces::srv::GetCameraInfo>(
            camera_name + "/get_camera_info",
            std::bind(&CamServer::camera_info_callback, this, _1, _2, _3));

        RCLCPP_INFO(
            node_->get_logger(),
            "Camera server init finished.");
    }

    void CamServer::cam_info_timer_callback()
    {
        info_pub_->publish(camera_info_manager_->getCameraInfo());
    }

    void CamServer::timer_callback()
    {
        if (cam_interface_->grab_img(img_, time_stamp_ms_))
        {
            auto header = std_msgs::msg::Header();
            header.stamp = rclcpp::Clock().now();

            // publish image msg

            // use loaned message
            auto img_msg = img_pub_->borrow_loaned_message();
            cv_bridge::CvImage(header, "bgr8", img_).toImageMsg(img_msg.get());
            img_pub_->publish(std::move(img_msg));

            // default message
            // sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
            //                                                  header, "bgr8", img_)
            //                                                  .toImageMsg();
            // img_pub_->publish(*img_msg);
        }
        else
        {
            // try to reopen camera
            using namespace std::chrono_literals;
            if (reopen_cnt_ % 5 == 0)
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
        const rm_interfaces::srv::GetCameraInfo::Request::SharedPtr request,
        rm_interfaces::srv::GetCameraInfo::Response::SharedPtr response)
    {
        (void)request_header;
        (void)request;

        if (camera_info_manager_->isCalibrated())
        {
            response->camera_info = camera_info_manager_->getCameraInfo();
            response->success = true;
        }
        else {
            response->success = false;
        }
    }
} // namespace rm_cam
