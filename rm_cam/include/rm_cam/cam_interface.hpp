#ifndef RM_CAM__CAM_INTERFACE_HPP
#define RM_CAM__CAM_INTERFACE_HPP

#include <unordered_map>
#include "opencv2/opencv.hpp"

namespace rm_cam
{

    // 摄像头参数枚举类
    enum class CamParam
    {
        Width,
        Height,
        AutoExposure,
        Exposure,
        Brightness,
        AutoWhiteBalance,
        WhiteBalance,
        Gain,
        Gamma,
        Contrast,
        Saturation,
        Hue,
        Fps,
        RGain,
        GGain,
        BGain
    };

    /** @brief 相机接口基类
     * 
     **/
    class CamInterface
    {
    public:
        // 基础功能

        /**
         * @brief 打开相机
         * 
         * @return true
         * @return false
         */
        virtual bool open() = 0;

        /**
         * @brief 关闭相机
         * 
         * @return true 
         * @return false
         */
        virtual bool close() = 0;

        /**
         * @brief 是否打开相机
         * 
         * @return true 
         * @return false
         */
        virtual bool is_open() = 0;

        /**
         * @brief 获取一帧图像
         * 
         * @param image 输出图像
         * @return true 
         * @return false 
         */
        virtual bool grab_img(cv::Mat &image) = 0;

        /**
         * @brief 获取一帧图像及其时间戳
         * 
         * @param image 输出图像
         * @param timestamp_ms 时间戳单位ms
         * @return true 
         * @return false 
         */
        virtual bool grab_img(cv::Mat &image, double &timestamp_ms) = 0;

        // 设置摄像头参数
        /**
         * @brief 设置参数
         * 
         * @param Type 参数类型
         * @param value 参数值
         * @return true 
         * @return false 
         */
        virtual bool set_parameter(CamParam Type, int value)
        {
            if (_param.find(Type) != _param.end())
            {
                _param[Type] = value;
                return true;
            }
            else
            {
                return false;
            }
        }

        // 获取摄像头参数
        /**
         * @brief 获取参数
         * 
         * @param Type 参数类型
         * @param value 参数值
         * @return true 
         * @return false 
         */
        virtual bool get_parameter(CamParam Type, int &value)
        {
            if (_param.find(Type) != _param.end())
            {
                value = _param[Type];
                return true;
            }
            else
            {
                return false;
            }
        }

    protected:
        std::unordered_map<CamParam, int> _param; // 摄像头参数
    };

} // namespace rm_cam

#endif // RM_CAM__CAM_INTERFACE_HPP
