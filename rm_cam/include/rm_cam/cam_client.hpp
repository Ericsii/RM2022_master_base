#ifndef RM_CAM__CAM_CLIENT_HPP
#define RM_CAM__CAM_CLIENT_HPP

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "opencv2/opencv.hpp"


namespace rm_cam
{
using CallBack = std::function<void (cv::Mat&, double)>;

class CamClient
{
public:
    CamClient() = delete;

    /**
     * @brief Construct a new Cam Client object
     * 
     * @param node rclcpp节点
     * @param camera_name 相机topic名称
     * @param process_fn 图像处理回调函数std::function<void (cv::Mat&, double)>
     * @param spin_thread 开启线程
     */
    CamClient(
        rclcpp::Node::SharedPtr node,
        const std::string &camera_name,
        CallBack process_fn,
        bool spin_thread = true
    );
    ~CamClient();

    /**
     * @brief Get the camera info object
     * 
     * @param info 返回的camera info
     */
    bool get_camera_info(sensor_msgs::msg::CameraInfo &info);
    
    /**
     * @brief 开启图像处理
     * 
     */
    void start();

    /**
     * @brief 关闭图像处理
     * 
     */
    void stop();

private:
    void img_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);

private:
    rclcpp::Node::SharedPtr                                     node_;                      // rcl节点
    std::string                                                 camera_name_;               // 相机topic名称
    rclcpp::CallbackGroup::SharedPtr                            callback_group_{nullptr};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr        executor_;
    std::unique_ptr<std::thread>                                executor_thread_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr    img_sub_;
    CallBack                                                    process_fn_;                // 图像处理回调函数
    bool                                                        spin_thread_;               // 线程开关
    bool                                                        run_flag_{false};           // 图像处理开关
};
} // namespace rm_cam

#endif // RM_CAM__CAM_CLIENT_HPP