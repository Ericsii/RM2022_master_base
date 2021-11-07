#include "rm_cam/cam_client.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "cv_bridge/cv_bridge.h"
#include "rm_interfaces/srv/get_camera_info.hpp"
#include "rm_interfaces/qos_policy.hpp"


namespace rm_cam
{
CamClient::CamClient(
    rclcpp::Node::SharedPtr node,
    const std::string &camera_name,
    CallBack process_fn,
    bool spin_thread
): node_(node), camera_name_(camera_name), process_fn_(process_fn), spin_thread_(spin_thread)
{
    using namespace std::placeholders;
    node_->declare_parameter("best_effort_qos", false);
    auto best_effort_qos = node_->get_parameter("best_effort_qos").as_bool();

    if (spin_thread_)
    {
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        if (best_effort_qos)
        {
            // 图像传输QoS配置
            rclcpp::QoS img_pub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
            img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            camera_name_ + "/image_raw", img_pub_qos_profile, std::bind(&CamClient::img_cb, this, _1), sub_opt);
        }
        else {
            img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            camera_name_ + "/image_raw", 1, std::bind(&CamClient::img_cb, this, _1), sub_opt);
        }
        
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
        executor_thread_ = std::make_unique<std::thread>([&](){executor_->spin();});
    }
    else {
        if (best_effort_qos)
        {
            // 图像传输QoS配置
            rclcpp::QoS img_pub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
            img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            camera_name_ + "/image_raw", img_pub_qos_profile, std::bind(&CamClient::img_cb, this, _1));
        }
        else {
            img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            camera_name_ + "/image_raw", 1, std::bind(&CamClient::img_cb, this, _1));
        }
    }
}

CamClient::~CamClient()
{
    if(spin_thread_)
    {
        executor_->cancel();
        executor_thread_->join();
    }
}

void CamClient::img_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (run_flag_)
    {
        auto img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        auto img_stamp = msg->header.stamp.sec + 0.000000001 * msg->header.stamp.nanosec;
        process_fn_(img, img_stamp);
    }
}

bool CamClient::get_camera_info(sensor_msgs::msg::CameraInfo &info)
{
    using namespace std::chrono_literals;
    auto callback_group = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto client = node_->create_client<rm_interfaces::srv::GetCameraInfo>(
        camera_name_, rmw_qos_profile_services_default, callback_group
    );
    exec->add_callback_group(callback_group, node_->get_node_base_interface());
    
    while(!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[get_camera_info] client interrupted!"
            );
            return false;
        }
    }
    auto request = std::make_shared<rm_interfaces::srv::GetCameraInfo::Request>();
    auto result_future = client->async_send_request(request);
    if (exec->spin_until_future_complete(result_future, 5s) != 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[get_camera_info] service timeout!"
        );
        return false;
    }

    auto result = result_future.get();
    if (result->success)
    {
        info = result->camera_info;
        return true;
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "[get_camera_info] service call failed.");
        return false;
    }
}

void CamClient::start() {run_flag_ = true;}

void CamClient::stop() {run_flag_ = false;}

} // namespace rm_cam
