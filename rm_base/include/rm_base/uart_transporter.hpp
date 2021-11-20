#ifndef RM_BASE_UART__TRANSPORTER_INTERFACE_HPP_
#define RM_BASE_UART__TRANSPORTER_INTERFACE_HPP_

#include <string>
#include "rm_base/transporter_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_base
{
    //UART Serial data transport device
    class UartTransporter : public TransporterInterface
    {
    public:
        UartTransporter(
            const std::string &device_path = "/dev/ttyACM1",
            int speed = 1152000,
            rclcpp::Node::SharedPtr node = nullptr,
            int flow_ctrl = 0,
            int databits = 8,
            int stopbits = 1,
            int parity = 'N')
            : device_path_(device_path), speed_(speed), node_(node), flow_ctrl_(flow_ctrl),
              databits_(databits), stopbits_(stopbits), parity_(parity) {}

        /**
         * @brief 打开串口
         * 
         * @return true 
         * @return false 
         */
        bool open() override;

        /**
         * @brief 关闭串口
         */
        void close() override;

        /**
         * @brief 返回串口开启参数
         * 
         * @return true 
         * @return false 
         */
        bool is_open() override;

        /**
         * @brief 读取串口数据到buffer中，Read NBYTES into BUF from FD. 
         * 
         * @param buffer 
         * @param len 
         * @return 【int】 the number read, -1 for errors or 0 for EOF.
         */
        int read(void *buffer, size_t len) override;

        /**
         * @brief 将buffer中数据写入串口，Write N bytes of BUF to FD.
         * 
         * @param buffer 
         * @param len 
         * @return 【int】 the number written, or -1.
         */
        int write(const void *buffer, size_t len) override;

    private:
        /**
         * @brief Set the param object
         * 
         * @param speed 
         * @param flow_ctrl 
         * @param databits 
         * @param stopbits 
         * @param parity 
         * @return true 
         * @return false 
         */
        bool set_param(
            int speed = 115200, int flow_ctrl = 0, int databits = 8,
            int stopbits = 1, int parity = 'N');

    private:
        //设备参数
        std::string device_path_; //设备名
        int speed_;               //波特率
        rclcpp::Node::SharedPtr node_;
        int flow_ctrl_;           //数据流控制
        int databits_;            //数据位
        int stopbits_;            //停止位
        int parity_;              //校验位
        bool is_open_{false};     //设备状态
        int fd_{-1};              //文件描述符(0 in /1 out)
    };
}

#endif // RM_BASE_UART__TRANSPORTER_INTERFACE_
