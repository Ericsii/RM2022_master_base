#include "rm_cam/virtual_cam.hpp"

#include <string>

namespace rm_cam
{
    VirtualCam::VirtualCam(int mode, const std::string &path)
    {
        _param[CamParam::Width] = 0;
        _param[CamParam::Height] = 0;
        _param[CamParam::Fps] = 0;
        if (mode == IMAGE_MODE)
        {
            path_ = path;
            current_mode_ = IMAGE_MODE;
        }
        else if (mode == VIDEO_MODE)
        {
            path_ = path;
            current_mode_ = VIDEO_MODE;
        }
        is_open_ = false;
    }

    bool VirtualCam::open()
    {
        if (current_mode_ == IMAGE_MODE)
        {
            img_ = cv::imread(path_);
            if (!img_.empty())
            {
                _param[CamParam::Width] = img_.size().width;
                _param[CamParam::Height] = img_.size().height;
                is_open_ = true;
                return true;
            }
        }
        else if (current_mode_ == VIDEO_MODE)
        {
            if (cap_.open(path_))
            {
                _param[CamParam::Width] = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
                _param[CamParam::Height] = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
                _param[CamParam::Fps] = cap_.get(cv::CAP_PROP_FPS);
                total_frames_ = cap_.get(cv::CAP_PROP_FRAME_COUNT);
                current_frame_ = 0;
                is_open_ = true;
                return true;
            }
        }
        return false;
    }

    bool VirtualCam::close()
    {
        if (current_mode_ == IMAGE_MODE)
        {
            return true;
        }
        else if (current_mode_ == VIDEO_MODE)
        {
            cap_.release();
            return true;
        }
        return false;
    }

    bool VirtualCam::is_open() { return is_open_; }

    bool VirtualCam::grab_img(cv::Mat &image)
    {
        if (is_open_)
        {
            if (current_mode_ == IMAGE_MODE)
            {
                image = img_.clone();
                return true;
            }
            else if (current_mode_ == VIDEO_MODE)
            {
                if (cap_.read(image))
                {
                    current_frame_++;
                    if (current_frame_ > total_frames_ - 2)
                    {
                        current_frame_ = 0;
                        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                    }
                    return true;
                }
            }
        }
        return false;
    }
} // namespace rm_cam
