#ifndef RM_CAM__MINDVISION_CAM_HPP
#define RM_CAM__MINDVISION_CAM_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "CameraApi.h"
#include "opencv2/opencv.hpp"
#include "rm_cam/cam_interface.hpp"

namespace rm_cam
{
    // MindVision相机类
    class MindVisionCam: public CamInterface
    {
    public:
        /** Construct
         * @param camera_sn 相机SN号
         * @param node ROS节点指针
         * @param config_path 相机配置文件路径
         **/
        explicit MindVisionCam(
            const std::string &camera_sn,
            rclcpp::Node::SharedPtr node,
            const std::string &config_path = ""
        );


        bool open() override;
        bool close() override;
        bool is_open() override;
        bool grab_img(cv::Mat &image) override;
        bool grab_img(cv::Mat &image, double &timestamp_ms) override;
        bool save_config(const std::string &path);

    private:
        bool read_camera_info();
        bool config_camera();

    private:
        std::string         camera_sn_;         // 相机sn号
        
        rclcpp::Node::SharedPtr node_;
        std::string         config_path_;       // 相机配置文件路径

        int                 hCamera_;           // 相机描述子
        tSdkCameraDevInfo   cameraList_[16];    // 相机列表(仅获取16个相机)
        tSdkCameraCapbility cameraInfo_;        // 相机特征信息

        BYTE*               pFrameBuffer_;      // RAW图像缓存
        bool                is_open_;           // 相机状态
    };
} // namespace rm_cam


#endif // RM_CAM__MINDVISION_CAM_HPP