#include "rm_base/uart_transporter.hpp"

#include <stdio.h>  /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/

namespace rm_base
{
    //设置串口参数
    bool UartTransporter::set_param(int speed, int flow_ctrl, int databits, int stopbits, int parity)
    {
        int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
        int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};
        struct termios options;

        //将fd串口对象的参数保存与options中，调用成功则返回0
        if (tcgetattr(fd_, &options) != 0)
        {
            perror("Setup serial err!!!");
            return false;
        }

        //设置串口波特率
        for (size_t i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
        {
            if (speed == name_arr[i])
            {
                cfsetispeed(&options, speed_arr[i]);
                cfsetospeed(&options, speed_arr[i]);
            }
        }

        //修改options中的串口控制模式，保证程序不占用串口，保证能从串口中读数据
        options.c_cflag |= CLOCAL;
        options.c_cflag |= CREAD;

        //设置数据流控制
        switch (flow_ctrl)
        {
        case 0: // 不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1: // 使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2: // 使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
        }

        //设置数据位，屏蔽其他标志位
        options.c_cflag &= ~CSIZE;
        switch (databits)
        {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr, "Unsupported data size!!!\n");
        }

        //设置校验位
        switch (parity)
        {
        case 'n':
        case 'N': // 无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O': // 设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E': // 设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': // 设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
        }

        // 设置停止位
        switch (stopbits)
        {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported stop bits\n");
        }

        // 修改输出模式，原始数据输出
        options.c_oflag &= ~OPOST;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // 允许传输特殊字符，否则特殊字符0x0d,0x11,0x13会被屏蔽或映射。
        // 0x0d 回车符CR
        // 0x11 ^Q VSTART字符
        // 0x13 ^S VSTOP字符
        options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

        // 设置等待时间和最小接收字符
        options.c_cc[VTIME] = 1; // 读取一个字符等待 1 * 0.1s
        options.c_cc[VMIN] = 1;  // 读取字符的最少个数为 1
        tcflush(fd_, TCIFLUSH);  // 清空终端未完成的输入/输出请求及数据。

        // 激活配置的参数,将修改后的termios数据options设置到串口中
        if (tcsetattr(fd_, TCSANOW, &options) != 0)
        {
            //tcsetattr用于设置终端的相关参数, 如果返回为-1或是出现EINTR错误，则表示参数设置错误
            //TCSANOW：不等数据传输完毕就立即改变属性。
            perror("com set error!\n");
        }
        return true;
    }

    //开启串口
    bool UartTransporter::open()
    {
        if (is_open_)
        {
            return true;
        }
        fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //打开设备
        if (fd_ == -1)
        {
            printf("cannot open uart device: %s\n", device_path_.c_str());
            return false;
        } else{
            printf("Open uart device: %s\n", device_path_.c_str());
        }

        // 串口设置为阻塞状态
        if (fcntl(fd_, F_SETFL, 0) < 0)
        {
            printf("fcntl failed!\n");
            return false;
        }
        // 测试是否为终端设备
        if (0 == isatty(STDIN_FILENO))
        {
            printf("standard input is not a end device\n");
            return false;
        }
        // 设置串口数据帧格式
        if (set_param(speed_, flow_ctrl_, databits_, stopbits_, parity_))
        {
            printf("set param\n");
            return false;
        }
        is_open_ = true;
        return true;
    }

    //关闭串口
    void UartTransporter::close()
    {
        if (!is_open_)
        {
            return;
        }
        ::close(fd_);
        fd_ = -1;
        is_open_ = false;
    }

    //返回串口开启参数
    bool UartTransporter::is_open() { return is_open_; }

    //读取串口数据到buffer中
    int UartTransporter::read(void *buffer, size_t len)
    {
        int ret = ::read(fd_, buffer, len);
        tcflush(fd_, TCIFLUSH);
        return ret;
    }

    //将buffer中数据写入串口
    int UartTransporter::write(const void *buffer, size_t len)
    {
        int ret = ::write(fd_, buffer, len);
        return ret;
    }

}
