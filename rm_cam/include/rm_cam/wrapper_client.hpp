#ifndef RM_AUTO_AIM__AIM_CAM_CLIENT_HPP
#define RM_AUTO_AIM__AIM_CAM_CLIENT_HPP

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "opencv2/opencv.hpp"

namespace rm_cam
{
    // 回调函数类
    using DataCallBack = std::function<void(std_msgs::msg::Header, cv::Mat &, geometry_msgs::msg::Quaternion)>;

    using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Imu>;

    class WrapperClient
    {
    public:
        WrapperClient() = delete;

        /**
         * @brief Construct a new Wrapper Client object
         * 
         * @param node rclcpp节点
         * @param camera_name 相机topic名称
         * @param imu_name imu topic名称
         * @param process_fn 数据处理回调函数
         */
        WrapperClient(
            rclcpp::Node::SharedPtr node,
            const std::string &camera_name,
            const std::string &imu_name,
            DataCallBack process_fn);
        ~WrapperClient() = default;

        /**
         * @brief Get the camera info object
         * 
         * @param info 返回的camera info
         */
        bool get_camera_info(sensor_msgs::msg::CameraInfo &info);

        /**
         * @brief 开启回调处理
         * 
         */
        void start();

        /**
         * @brief 关闭回调处理
         * 
         */
        void stop();

    private:
        void data_cbk(sensor_msgs::msg::Image::ConstSharedPtr img,
                      sensor_msgs::msg::Imu::ConstSharedPtr pose);

    private:
        rclcpp::Node::SharedPtr node_; // rclcpp 节点
        std::string camera_name_;      // 相机 topic 名称
        std::string imu_name_;         // IMU topic 名称
        DataCallBack process_fn_;      // 回调函数
        bool run_flag_{false};         // 回调开关

        message_filters::Subscriber<sensor_msgs::msg::Image> img_sub_;           // 图像订阅
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;   // imu 订阅
        std::shared_ptr<message_filters::Synchronizer<ApproximatePolicy>> sync_; // 同步订阅器
    };
} // namespace rm_auto_aim

#endif // RM_AUTO_AIM__AIM_CAM_CLIENT_HPP