#ifndef RM_BASE_UART__TRANSPORTER_INTERFACE_HPP_
#define RM_BASE_UART__TRANSPORTER_INTERFACE_HPP_

#include <string>
#include "rm_base/transporter_interface.hpp"

namespace rm_base
{
    //UART Serial data transport device
    class UartTransporter : public TransporterInterface
    {
    public:
        UartTransporter(
            const std::string &device_path = "/dev/ttyUSB0",
            int speed = 115200,
            int flow_ctrl = 0,
            int databits = 8,
            int stopbits = 1,
            int parity = 'N')
            : device_path_(device_path), speed_(speed), flow_ctrl_(flow_ctrl),
              databits_(databits), stopbits_(stopbits), parity_(parity) {}

        bool open() override;
        void close() override;
        bool is_open() override;
        int read(void *buffer, size_t len) override;
        int write(const void *buffer, size_t len) override;

    private:
        bool set_param(
            int speed = 115200, int flow_ctrl = 0, int databits = 8,
            int stopbits = 1, int parity = 'N');

    private:
        //设备参数
        std::string device_path_; //设备名
        int speed_;               //波特率
        int flow_ctrl_;           //数据流控制
        int databits_;            //数据位
        int stopbits_;            //停止位
        int parity_;              //校验位
        bool is_open_{false};     //设备状态
        int fd_{-1};              //文件描述符(0 in /1 out)
    };
}

#endif // RM_BASE_UART__TRANSPORTER_INTERFACE_
