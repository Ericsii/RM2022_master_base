#include "rm_cam/mindvision_cam.hpp"

#include <iostream>

#include "CameraApi.h"

namespace rm_cam
{

    // 相机构造函数
    /**
     * @brief Construct a new Mind Vision Cam:: Mind Vision Cam object
     * 
     * @param camera_sn 相机SN号
     * @param node RClNode
     * @param config_path 相机配置文件路径
     */
    MindVisionCam::MindVisionCam(
        const std::string &camera_sn,
        rclcpp::Node::SharedPtr node,
        const std::string &config_path
        ): camera_sn_(camera_sn), node_(node), config_path_(config_path)
    {
        _param[CamParam::Height] = 0;
        _param[CamParam::Width] = 0;
        _param[CamParam::Fps] = 0;
        _param[CamParam::AutoExposure] = 0;         // 0 为手动曝光; 其余为自动曝光亮度目标值
        _param[CamParam::Exposure] = 0;             // 曝光时间 单位 us (默认 2000us)
        _param[CamParam::WhiteBalance] = 0;
        _param[CamParam::RGain] = 0;
        _param[CamParam::GGain] = 0;
        _param[CamParam::BGain] = 0;
        _param[CamParam::Gamma] = 0;
        _param[CamParam::Contrast] = 0;             // TODO
        _param[CamParam::Saturation] = 0;
        _param[CamParam::Hue] = 0;                  // TODO

        is_open_ = false;
    }


    bool MindVisionCam::open()
    {
        if (is_open_)
        {
            if (!close()) return false;
        }

        CameraSdkStatus status; // MVSDK 状态变量

        // 获取前16个相机信息
        int camera_num = 3;
        status = CameraEnumerateDevice(cameraList_, &camera_num);
        if (status != CAMERA_STATUS_SUCCESS) 
        {
            RCLCPP_FATAL(
                node_->get_logger(),
                "ERROR:[%d] - No cammera was found!", status
            );
            return false;
        }
        
        // 初始化相机
        // (-1,-1)表示加载上次退出前保存的参数，如果是第一次使用该相机，则加载默认参数。
        bool is_init = false;
        for (auto &info: cameraList_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "CAMERA_SN: %s", info.acSn
            );
            if (info.acSn == camera_sn_)
            {
                status = CameraInit(&info, -1, -1, &hCamera_);
                if (status != CAMERA_STATUS_SUCCESS) 
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Camera initialize failed!"
                    );
                    return false;
                }
                is_init = true;
                break;
            }
        }
        if (!is_init)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Cannot find camera: [%s].", camera_sn_.c_str()
            );
            return false;
        }

        // 读取相机配置文件
        bool is_from_config = false;
        if (config_path_ != "")
        {
            status = CameraReadParameterFromFile(hCamera_, (char *)config_path_.c_str());
            if (status != CAMERA_STATUS_SUCCESS) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Read config file %s failed", config_path_.c_str()
                );
                return false;
            }

            is_from_config = true;
        }

        // 获取相机特征描述符
        CameraGetCapability(hCamera_, &cameraInfo_);

        if (is_from_config)
        {
            // 读取相机参数 输出成ROS param
            if (!read_camera_info()) return false;
        }
        else {
            // 手动设置
            if (!config_camera()) return false;
        }

        status = CameraPlay(hCamera_);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(
                    node_->get_logger(),
                    "MDSDK play failed!"
                );
            return false;
        }

        is_open_ = true;
        return true;
    }

    // 关闭相机 释放图像缓存
    bool MindVisionCam::close()
    {
        if (CameraUnInit(hCamera_) != CAMERA_STATUS_SUCCESS) return false;
        is_open_ = false;
        return true;
    }

    bool MindVisionCam::is_open()
    {
        return is_open_;
    }

    bool MindVisionCam::grab_img(cv::Mat &image)
    {
        if (!is_open())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Camera is not open!"
            );
            return false;
        }
        
        CameraSdkStatus status;
        tSdkFrameHead header;

        // 进行一次触发
        status = CameraSoftTrigger(hCamera_);

        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "ERROR: [%d] - Trigger failed!", status
            );
        }

        // 读取帧 超时100ms
        status = CameraGetImageBuffer(hCamera_, &header, &pFrameBuffer_, 100);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Frame read failed!"
            );
        }

        image = cv::Mat(header.iHeight, header.iWidth, CV_8UC3);

        // 转换raw图像
        status = CameraImageProcess(hCamera_, pFrameBuffer_, image.data, &header);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(node_->get_logger(), "Image process failed!");
        }

        // 释放缓存
        CameraReleaseImageBuffer(hCamera_, pFrameBuffer_);

        return true;
    }

    bool MindVisionCam::grab_img(cv::Mat &image, double &timestamp_ms)
    {
        if (!is_open())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Camera is not open!"
            );
            return false;
        }
        
        CameraSdkStatus status;
        tSdkFrameHead header;
        
        // 进行一次触发
        status = CameraSoftTrigger(hCamera_);
        if (status == CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Trigger failed!"
            );
        }

        // 读取帧 超时100ms
        status = CameraGetImageBuffer(hCamera_, &header, &pFrameBuffer_, 100);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Frame read failed!"
            );
        }

        image = cv::Mat(header.iHeight, header.iWidth, CV_8UC3);
        // 转换raw图像
        status = CameraImageProcess(hCamera_, pFrameBuffer_, image.data, &header);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(node_->get_logger(), "Image process failed!");
        }
        // 时间戳
        timestamp_ms = header.uiTimeStamp / 10.;

        // 释放缓存
        CameraReleaseImageBuffer(hCamera_, pFrameBuffer_);

        return true;
    }

    // 保存相机配置文件
    bool MindVisionCam::save_config(const std::string &path)
    {
        if (CameraSaveParameterToFile(hCamera_, (char *)path.c_str()) != CAMERA_STATUS_SUCCESS)
            return false;
        return true;
    }

    // 读取相机信息
    bool MindVisionCam::read_camera_info()
    {

        // 分辨率
        int width, height;
        if (cameraInfo_.iImageSizeDesc > 0)
        {
            width = cameraInfo_.pImageSizeDesc->iWidth, 
            height = cameraInfo_.pImageSizeDesc->iHeight;
            this->set_parameter(CamParam::Width, width);
            this->set_parameter(CamParam::Height, height);
        }

        // 曝光
        BOOL ae_state;
        CameraGetAeState(hCamera_, &ae_state);
        if (ae_state)
        {
            // 自动曝光
            int ae_target;
            CameraGetAeTarget(hCamera_, &ae_target);
            this->set_parameter(CamParam::AutoExposure, ae_target);
        }
        else
        {
            // 手动曝光
            double exposure;
            CameraGetExposureTime(hCamera_, &exposure);
            this->set_parameter(CamParam::Exposure, static_cast<int>(exposure));
        }

        // RGB增益
        int Rgain, Ggain, Bgain;
        CameraGetGain(hCamera_, &Rgain, &Ggain, &Bgain);
        this->set_parameter(CamParam::RGain, Rgain);
        this->set_parameter(CamParam::GGain, Ggain);
        this->set_parameter(CamParam::BGain, Bgain);

        // 饱和度
        int saturation;
        CameraGetSaturation(hCamera_, &saturation);
        this->set_parameter(CamParam::Saturation, saturation);

        // 设置相机软触发 每次触发帧数固定1
        CameraSetTriggerMode(hCamera_, 1);
        CameraSetTriggerCount(hCamera_, 1);

        return true;
    }

    // 手动设置相机
    bool MindVisionCam::config_camera()
    {
        CameraSdkStatus status;
        // 判断黑白输出模式
        if (cameraInfo_.sIspCapacity.bMonoSensor) 
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Setting camera to mono sensor."
            );
            CameraSetIspOutFormat(hCamera_, CAMERA_MEDIA_TYPE_MONO8);
        }
        else {
            RCLCPP_INFO(
                node_->get_logger(),
                "Setting camera to RGB sensor."
            );
            CameraSetIspOutFormat(hCamera_, CAMERA_MEDIA_TYPE_BGR8);
        }

        // 设置相机软触发 每次触发帧数固定1
        CameraSetTriggerMode(hCamera_, 1);
        CameraSetTriggerCount(hCamera_, 1);

        // 曝光设置
        int auto_exposure, exposure;
        this->get_parameter(CamParam::AutoExposure, auto_exposure);
        this->get_parameter(CamParam::Exposure, exposure);
        if (auto_exposure == 0
        || static_cast<u_int>(auto_exposure) < cameraInfo_.sExposeDesc.uiTargetMin 
        || static_cast<u_int>(auto_exposure) > cameraInfo_.sExposeDesc.uiTargetMax) 
        {
            // 手动曝光
            CameraSetAeState(hCamera_, FALSE);
            if (exposure == 0)
            {
                exposure = 2000;
            }

            RCLCPP_INFO(
                node_->get_logger(),
                "Setting camera exposure: %dus.", exposure
            );

            status = CameraSetExposureTime(hCamera_, static_cast<double>(exposure));
            if (status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "ERROR: [%d] - Camera exposure set failed!", status
                );
                return false;
            }
            double final_exposure;
            CameraGetExposureTime(hCamera_, &final_exposure);
            this->set_parameter(CamParam::Exposure, static_cast<int>(final_exposure));
        }
        else {
            // 自动曝光设置
            CameraSetAeState(hCamera_, TRUE);
            // 设定自动曝光亮度目标值
            RCLCPP_INFO(
                node_->get_logger(),
                "Setting camera auto exposure: %d.", auto_exposure
            );
            CameraSetAeTarget(hCamera_, auto_exposure);
        }

        // RGB通道增益设置
        int Rgain, Ggain, Bgain;
        this->get_parameter(CamParam::RGain, Rgain);
        this->get_parameter(CamParam::GGain, Ggain);
        this->get_parameter(CamParam::BGain, Bgain);
        if (Rgain > 0 && Rgain < 400 
         && Ggain > 0 && Ggain < 400
         && Bgain > 0 && Bgain < 400)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Setting camera gain: R%d G%d B%d.", Rgain, Ggain, Bgain
            );
            status = CameraSetGain(hCamera_, Rgain, Ggain, Bgain);
            if (status != CAMERA_STATUS_SUCCESS) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Camera gain set failed!"
                );
                return false;
            }
        }

        // 饱和度设置
        int saturation;
        this->get_parameter(CamParam::Saturation, saturation);
        if (saturation != 0 && saturation >= cameraInfo_.sSaturationRange.iMin 
        && saturation <= cameraInfo_.sSaturationRange.iMax)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Setting camera saturation: %d.", saturation
            );
            status = CameraSetSaturation(hCamera_, saturation);
            if (status != CAMERA_STATUS_SUCCESS) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Camera saturation set failed!"
                );
                return false;
            }
        }

        // 以最大分辨率拍摄
        int width = cameraInfo_.sResolutionRange.iWidthMax, 
            height = cameraInfo_.sResolutionRange.iHeightMax;
        this->set_parameter(CamParam::Width, width);
        this->set_parameter(CamParam::Height, height);

        return true;
    }
} // namespace rm_cam
