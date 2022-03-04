#include "rm_cam/wrapper_client.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "cv_bridge/cv_bridge.h"
#include "rm_interfaces/srv/get_camera_info.hpp"
#include "rm_interfaces/qos_policy.hpp"

namespace rm_cam
{
    WrapperClient::WrapperClient(
        rclcpp::Node::SharedPtr node,
        const std::string &camera_name,
        const std::string &imu_name,
        DataCallBack process_fn) : node_(node), camera_name_(camera_name), imu_name_(imu_name), process_fn_(process_fn)
    {
        using std::placeholders::_1;
        using std::placeholders::_2;

        node_->declare_parameter("best_effort_qos", false);
        auto best_effort_qos = node_->get_parameter("best_effort_qos").as_bool();

        if (best_effort_qos)
        {
            // 图像传输配置
            rclcpp::QoS img_sub_qos_profile(rclcpp::KeepLast(2), best_effort_qos_policy);
            img_sub_.subscribe(node_, camera_name_ + "/image_raw", img_sub_qos_profile.get_rmw_qos_profile());

            // imu传输QoS配置
            rclcpp::QoS imu_sub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
            imu_sub_.subscribe(node_, imu_name_, imu_sub_qos_profile.get_rmw_qos_profile());
        }
        else
        {
            img_sub_.subscribe(node_, camera_name_ + "/image_raw");
            imu_sub_.subscribe(node_, imu_name_);
        }

        caminfo_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(camera_name_ + "/camera_info", 10, std::bind(&WrapperClient::cam_info_cbk, this, _1));
        sync_ = std::make_shared<message_filters::Synchronizer<ApproximatePolicy>>(ApproximatePolicy(5), img_sub_, imu_sub_);
        sync_->registerCallback(std::bind(&WrapperClient::data_cbk, this, _1, _2));
    }

    void WrapperClient::cam_info_cbk(sensor_msgs::msg::CameraInfo::ConstSharedPtr caminfo)
    {
        if (!read_info_)
        {
            cam_info_ = *caminfo;
            read_info_ = true;
        }
    }

    void WrapperClient::data_cbk(sensor_msgs::msg::Image::ConstSharedPtr img,
                                 sensor_msgs::msg::Imu::ConstSharedPtr pose)
    {
        if (run_flag_)
        {
            auto c_header = img->header;
            auto c_img = cv_bridge::toCvShare(img, "bgr8")->image.clone();
            auto c_pose = pose->orientation;
            process_fn_(c_header, c_img, c_pose);
        }
    }

    bool WrapperClient::get_camera_info(sensor_msgs::msg::CameraInfo &info)
    {
        using namespace std::chrono_literals;
        auto start = node_->now();

        while (!read_info_)
        {
            std::this_thread::sleep_for(200ms);
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "[get_camera_info] Client interrupted!");
                return false;
            }
            if ((node_->now() - start).seconds() > 5.0)
            {
                break;
            }
            RCLCPP_INFO(
                node_->get_logger(),
                "[get_camera_info] Trying to get camera info.");
        }

        if (read_info_)
        {
            info = cam_info_;
            return true;
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[get_camera_info] Timeout!");
            return false;
        }
    }

    void WrapperClient::start()
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Client start.");
        run_flag_ = true;
    }

    void WrapperClient::stop()
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Client stop.");
        run_flag_ = false;
    }
} // namespace rm_cam
