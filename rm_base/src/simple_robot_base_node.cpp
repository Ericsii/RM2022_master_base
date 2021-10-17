#include <thread>
#include <memory>
#include <iostream>
// using namespace std;

#include "rm_base/simple_robot_base_node.hpp"
#include "rm_base/uart_transporter.hpp"
#include "rm_interfaces/qos_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sstream>


namespace rm_base
{
    namespace protocol_example
    {
        typedef enum : unsigned char
        {
            GimbalAngleControl = 0x01,
            ChangeMode = 0xa1
        } ProtocolExample;
    }

    SimpleRobotBaseNode::SimpleRobotBaseNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("simple_robot_base", options);

        //SerialName 串口名参数配置
        node_->declare_parameter("serial_name", "/dev/ttyUSB0");
        node_->get_parameter("serial_name",this->serial_name);
        
        //串口收发参数配置
        node_->declare_parameter("serial_send", false);
        node_->declare_parameter("serial_recv", true);
        this->SerialSend = node_->get_parameter("serial_send").as_bool();
        this->SerialRecv = node_->get_parameter("serial_recv").as_bool();

        //初始化串口、数据包
        auto transporter = std::make_shared<rm_base::UartTransporter>(this->serial_name);
        this->packet_tool_ = std::make_shared<FixedPacketTool<16>>(transporter);

        //Qos配置
        node_->declare_parameter("custom_qos", false);
        auto custom_qos = node_->get_parameter("custom_qos").as_bool();
        
        
        //订阅cmd_gimbal topic, 发送数据到串口
        if (custom_qos)
        {
            //串口传输Qos配置
            rclcpp::QoS cmd_gimbal_sub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);

            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                "cmd_gimbal",
                cmd_gimbal_sub_qos_profile,
                std::bind(&SimpleRobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1));
        }
        else
        {
            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                "cmd_gimbal",
                10,
                std::bind(&SimpleRobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1));
        }  

        RCLCPP_INFO(node_->get_logger(), "serial_name: %s",this->serial_name.c_str());
        
        //串口数据接收线程
        listen_thread_ = std::make_unique<std::thread>(&SimpleRobotBaseNode::listen_loop, this);
        //参数配置线程
        param_set_thread_ = std::make_unique<std::thread>(&SimpleRobotBaseNode::param_set_loop, this);
    }

    //发送串口数据：yaw、pitch
    void SimpleRobotBaseNode::gimbal_cmd_cb(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
    {
        if(this->SerialSend)
        {
        FixedPacket<16> packet;
        packet.load_data<unsigned char>(protocol_example::GimbalAngleControl, 1);
        packet.load_data<unsigned char>(0x01, 2);
        packet.load_data<float>(msg->position.pitch, 3);
        packet.load_data<float>(msg->position.yaw, 7);

        //发送到串口的pitch（3-6）、yaw（7-10）值
        packet.set_check_byte(); //设置校验位字节（check_byte）为BCC校验码

        /*DEBUG---
        RCLCPP_INFO(node_->get_logger(), "pitch:%f,yaw:%f",msg->position.pitch,msg->position.yaw);

        uint8_t bcc;
        float Spitch, Syaw;
        packet.unload_data(Spitch, 3);
        packet.unload_data(Syaw, 7);
        packet.unload_data(bcc, 15);
        RCLCPP_INFO(node_->get_logger(), "SEND-PITCH packet '%f'",Spitch);
        RCLCPP_INFO(node_->get_logger(), "SEND-YAW packet: '%f'", Syaw);
        RCLCPP_INFO(node_->get_logger(), "SEND-BCC packet: '%x'",  bcc);

        const uint8_t *buf = packet.buffer();
        std::string debug_info;
        std::stringstream sstream;
        
        for(int i = 0; i < 16; ++i)
        {
            sstream << buf[i];
        }
        sstream >> debug_info;
        
        RCLCPP_INFO(node_->get_logger(), "%s", debug_info.c_str());
        ---DEBUG*/
        

        this->packet_tool_->send_packet(packet);
        
        //设定数据发送延迟
        std::this_thread::sleep_for(std::chrono::microseconds(5));
        }
    }

    //接收串口数据(电控部分)
    void SimpleRobotBaseNode::listen_loop()
    {
        FixedPacket<16> packet;
        while (rclcpp::ok())
        {
            if(this->SerialRecv)
            {
            if (this->packet_tool_->recv_packet(packet))
            {
                unsigned char cmd;
                packet.unload_data(cmd, 1);
                if (cmd == (unsigned char)protocol_example::GimbalAngleControl)
                {
                    unsigned char mode = 0;
                    packet.unload_data(mode, 2);
                    RCLCPP_INFO(node_->get_logger(), "RECV-cmd packet: '%x'", cmd);
                    RCLCPP_INFO(node_->get_logger(), "RECV-mode packet: '%x'", mode);
                    if (mode == 0x00)
                    {
                        RCLCPP_INFO(node_->get_logger(), "change mode: normal mode");
                        this->SerialSend = false;
                    }
                    else if (mode == 0x01)
                    {
                        RCLCPP_INFO(node_->get_logger(), "change mode: auto aim mode");
                        this->SerialSend = true;
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(), "change mode: mode err!!!");
                    }
                }
            }
            }
        }
    }

    void SimpleRobotBaseNode::param_set_loop()
    {
        
        std::string serial_name_temp;
        while (rclcpp::ok())
        {  
            //串口名改变
            node_->get_parameter("serial_name",serial_name_temp);
            /*DEBUG--- 
            RCLCPP_INFO(node_->get_logger(), "serialName: %s，%s", serial_name_temp.c_str(), this->serial_name.c_str()); 
            ---DEBUG*/
            auto transporter_temp = std::make_shared<rm_base::UartTransporter>(serial_name_temp);
            if(serial_name_temp != this->serial_name)  
            {
                this->serial_name = serial_name_temp;
                RCLCPP_INFO(node_->get_logger(), "serialName change to %s", serial_name_temp.c_str());   
                this->packet_tool_ = std::make_shared<FixedPacketTool<16>>(transporter_temp);
            }
            //节点收发功能开关
            this->SerialSend = node_->get_parameter("serial_send").as_bool();
            this->SerialRecv = node_->get_parameter("serial_recv").as_bool();
        }
            
    }
}

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(rm_base::SimpleRobotBaseNode)