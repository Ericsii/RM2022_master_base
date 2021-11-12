#include "rm_base/simple_robot_base_node.hpp"
#include "rm_base/uart_transporter.hpp"
#include "rm_interfaces/qos_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>

namespace rm_base
{
    namespace frame_type
    {
        typedef enum : unsigned char
        {
            ChangeMode = 0xa1,
            ChangeShootSpeed= 0xb1,
            ChangeColor = 0xc1,
            GimbalAngleControl= 0xd1
        } frame_type;
    }

    SimpleRobotBaseNode::SimpleRobotBaseNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("simple_robot_base", options);

        /*----------------------- Param ros参数 -----------------------*/
        //param参数：serial_name【串口名】
        node_->declare_parameter("serial_name", "/dev/ttyUSB0");
        node_->get_parameter("serial_name",this->serial_name);

         //param参数：serial_bps【串口波特率】
        node_->declare_parameter("serial_bps", 115200);
        node_->get_parameter("serial_bps", this->serial_bps);        

        //初始化串口、数据包
        auto transporter = std::make_shared<UartTransporter>(this->serial_name, this->serial_bps, node_);
        this->packet_tool_ = std::make_shared<FixedPacketTool<32>>(transporter);
        
        //param参数：serial_recv/serial_send【串口收发开关】
        node_->declare_parameter("serial_send", false);
        node_->declare_parameter("serial_recv", false);
        this->SerialSend = node_->get_parameter("serial_send").as_bool();
        this->SerialRecv = node_->get_parameter("serial_recv").as_bool();

        //param参数：custom_qos【Qos节点】
        node_->declare_parameter("custom_qos", false);
        auto custom_qos = node_->get_parameter("custom_qos").as_bool();

        //param参数：debug【DEBUG调试】
        node_->declare_parameter("debug", true);
        this->debug = node_->get_parameter("debug").as_bool();
        
        /*----------------------- Topic ros话题 -----------------------*/
        if (custom_qos)
        {
            //串口传输Qos配置
            rclcpp::QoS cmd_gimbal_sub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
            rclcpp::QoS gyro_attitude_pub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);

            //topic订阅：cmd_gimbal, 云台控制订阅，并将数据发送到串口
            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                "cmd_gimbal",
                cmd_gimbal_sub_qos_profile,
                std::bind(&SimpleRobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1)
                );
            //topic发布：gyro_attitude, 陀螺仪姿态发布
            gyro_attitude_pub_ = node_->create_publisher<rm_interfaces::msg::GyroAttitude>(
                "gyro_attitude",
                gyro_attitude_pub_qos_profile);
        }
        else
        {
            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                "cmd_gimbal",
                10,
                std::bind(&SimpleRobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1)
                );
            gyro_attitude_pub_ = node_->create_publisher<rm_interfaces::msg::GyroAttitude>(
                "gyro_attitude",
                10);
        }  

        /*----------------------- Service ros服务 -----------------------*/
        //service服务端：获取模式
        get_mode_srv_ = node_->create_service<rm_interfaces::srv::GetMode>(
            "get_mode", 
            // &SimpleRobotBaseNode::ModeGet);
            std::bind(&SimpleRobotBaseNode::ModeGet, this, std::placeholders::_1, std::placeholders::_2)
            );

        //service服务端：获取颜色
        get_color_srv_ = node_->create_service<rm_interfaces::srv::GetColor>(
            "get_color",
            std::bind(&SimpleRobotBaseNode::ColorGet, this, std::placeholders::_1, std::placeholders::_2)
            );

        //service服务端：获取射速
        get_shoot_speed_srv_ = node_->create_service<rm_interfaces::srv::GetShootSpeed>(
            "get_shoot_speed",
            std::bind(&SimpleRobotBaseNode::ShootSpeedGet, this, std::placeholders::_1, std::placeholders::_2)
            );

        if(this->debug)
            RCLCPP_INFO(node_->get_logger(), "Serial:%s(%d) Init ", this->serial_name.c_str(), this->serial_bps);
            if(this->SerialRecv)
                RCLCPP_INFO(node_->get_logger(), "SerialRecv ON!");
            if(this->SerialSend)
                RCLCPP_INFO(node_->get_logger(), "SerialSend ON!");
            if(custom_qos)
                RCLCPP_INFO(node_->get_logger(), "Qos ON!");
            RCLCPP_INFO(node_->get_logger(), "Service Mode/Color/Shoot-Speed start!");

        /*----------------------- Thread 线程 -----------------------*/
        //thread线程：串口数据接收
        listen_thread_ = std::make_unique<std::thread>(&SimpleRobotBaseNode::listen_loop, this);

        //thread线程:参数配置
        param_set_thread_ = std::make_unique<std::thread>(&SimpleRobotBaseNode::param_set_loop, this);
    }

    //topic订阅返回函数：发送串口数据
    void SimpleRobotBaseNode::gimbal_cmd_cb(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
    {
        if(this->SerialSend)
        {
            this->tid++;
            //----第一次发送----//
            if (this->debug)
            {
                if (this->tid == 1)
                    RCLCPP_INFO(node_->get_logger(), "Serial:%s(%d) Send Init ", this->serial_name.c_str(), this->serial_bps);
                else
                    RCLCPP_INFO(node_->get_logger(), "packet[%d] Send ", this->tid);
            }

            //----将数据导入数据包中----//
            FixedPacket<32> packet;
            packet.load_data<uint32_t>(this->tid, 1);
            packet.load_data<float>(msg->position.yaw, 5);
            packet.load_data<float>(msg->position.pitch, 9);

            /** RMUA
            packet.load_data<float>(msg->velocity.yaw, 9);
            packet.load_data<float>(msg->velocity.pitch, 9);
            packet.load_data<float>(0x00, 9);
            **/

            if(this->debug)
            {
                // packet.load_data<unsigned char>(frame_type::ChangeMode, 5);
                // packet.load_data<unsigned char>(0xaa, 6);

                // packet.load_data<unsigned char>(frame_type::ChangeShootSpeed, 5);
                // packet.load_data<int>(18, 6);

                // packet.load_data<unsigned char>(frame_type::ChangeColor, 5);
                // packet.load_data<unsigned char>(0xbb, 6);

                // packet.load_data<unsigned char>(frame_type::GimbalAngleControl, 5);
                // packet.load_data<uint32_t>(this->tid, 6);
                // packet.load_data<float>(1.0, 10);
                // packet.load_data<float>(2.0, 14);
                // packet.load_data<float>(3.0, 18);
                // packet.load_data<float>(4913.656, 22);

                // uint8_t bcc = 0x11;
                uint32_t id = 0;
                // float Spitch, Syaw;
                // packet.unload_data(Spitch, 7);
                // packet.unload_data(Syaw, 11);
                // RCLCPP_INFO(node_->get_logger(), "SEND-PITCH packet '%f'",Spitch);
                // RCLCPP_INFO(node_->get_logger(), "SEND-YAW packet: '%f'", Syaw);
                if(packet.unload_data(id, 1))
                    RCLCPP_INFO(node_->get_logger(), "SEND-ID: '%d'",  id);
                // if(packet.unload_data(bcc, 30))
                //     RCLCPP_INFO(node_->get_logger(), "SEND-BCC: '%x'",  bcc);
            }      

            packet.set_check_byte();        //设置校验位字节（check_byte）为BCC校验码
            bool send_ack = this->packet_tool_->send_packet(packet);
            if (!send_ack)
                RCLCPP_INFO(node_->get_logger(), "Serial Send Fail!!!");            
            
            //设定数据发送延迟
            // std::this_thread::sleep_for(std::chrono::microseconds(1));

            //清理缓存
            packet.clear();
        }
    }

    // service处理函数：模式切换
    void SimpleRobotBaseNode::ModeGet(const std::shared_ptr<rm_interfaces::srv::GetMode::Request> request,
                                        std::shared_ptr<rm_interfaces::srv::GetMode::Response> response)
    {
        response->mode = this->mode;
        std::string node_type = request->node_type;
        if(this->debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Get Mode Client: %s", node_type.c_str());
            if(this->mode == 1)
                RCLCPP_INFO(node_->get_logger(), "【Auto Aim】 Mode!");
            else if (this->mode == 2)
                RCLCPP_INFO(node_->get_logger(), "【Smell Power Lune】 Mode!");
            else if (this->mode == 3)
                RCLCPP_INFO(node_->get_logger(), "【Big Power Lune】 Mode!");
            else
                RCLCPP_INFO(node_->get_logger(), "【Normal】 Mode.");
        }
    }

    void SimpleRobotBaseNode::ColorGet(const std::shared_ptr<rm_interfaces::srv::GetColor::Request> request, 
                                            std::shared_ptr<rm_interfaces::srv::GetColor::Response> response)
    {
        response->color = this->color;
        std::string node_type = request->node_type;
        if(this->debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Get Color Client: %s", node_type.c_str());
            if(this->color == 0)
                RCLCPP_INFO(node_->get_logger(), "Ours Color is 【BLUE】!");
            else
                RCLCPP_INFO(node_->get_logger(), "Ours Color is 【RED】!");
        }
    }

    void SimpleRobotBaseNode::ShootSpeedGet(const std::shared_ptr<rm_interfaces::srv::GetShootSpeed::Request> request,
                            std::shared_ptr<rm_interfaces::srv::GetShootSpeed::Response> response)
    {
        response->shoot_speed = this->shoot_speed;
        std::string node_type = request->node_type;
        if(this->debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Get Color Client: %s", node_type.c_str());
            RCLCPP_INFO(node_->get_logger(), "Robot Shoot Speed is 【%d m/s】!", this->shoot_speed);
        }
    }

    // 接收串口数据(电控部分)
    void SimpleRobotBaseNode::listen_loop()
    {
        FixedPacket<32> packet;
        uint32_t recv_tid = 0;
        while (rclcpp::ok())
        {
            if(this->SerialRecv)
            {
                if (this->packet_tool_->recv_packet(packet))
                {
                    //----接收包编号----//
                    /*recv_tid【1-4】:包编号位 0x00000000-0xffffffff*/
                    packet.unload_data(recv_tid, 1);
                    if(this->debug)
                        RCLCPP_INFO(node_->get_logger(), "RECV packet [%d]", recv_tid);

                    //------接收包种类------//
                    /*cmd【5】:帧种类位 0xa1云台控制 0xb1射速改变 0xc1模式改变*/
                    unsigned char cmd;             
                    packet.unload_data(cmd, 5);
                    if (this->debug)
                        RCLCPP_INFO(node_->get_logger(), "RECV-cmd: '%x'", cmd);
                    
                    //---- 一、模式切换: mode【6】 ----//
                    if (cmd == (unsigned char)frame_type::ChangeMode)
                    {
                        /*mode【6】:模式切换位 0xaa自瞄 0xbb小能量机关 0xcc大能量机关*/
                        unsigned char mode = 0x00;
                        packet.unload_data(mode, 6);
                        if(this->debug)
                            RCLCPP_INFO(node_->get_logger(), "[Mode Change] RECV-package");
                            RCLCPP_INFO(node_->get_logger(), "RECV-mode: '%x'", mode); 

                        // if ((mode == 0xaa)||(mode == 0xbb)||(mode == 0xcc)) 
                        //     this->SerialSend = true;    //开启发送部分，开始向下位机发送数据     
                        // else
                        //     this->SerialSend = false;   //关闭发送部分，正常模式
                         
                        if (mode == 0xaa){
                            this->mode = 1;
                            RCLCPP_INFO(node_->get_logger(), "Mode：【Auto Aim】");
                        } else if (mode == 0xbb){
                            this->mode = 2;
                            RCLCPP_INFO(node_->get_logger(), "Mode：【Smell Nashor】");
                        } else if (mode == 0xcc){
                            this->mode = 3;
                            RCLCPP_INFO(node_->get_logger(), "Mode：【Big Nashor】");
                        } else {
                            this->mode = 0;
                            RCLCPP_INFO(node_->get_logger(), "Mode：【Normal】");
                        }
                    }
                    //---- 二、获取射速: shoot_speed【6-9】 ----//
                    if (cmd == (unsigned char)frame_type::ChangeShootSpeed)
                    {
                        if(this->debug)
                            RCLCPP_INFO(node_->get_logger(), "[Shoot-speed Get] RECV-package");
                        int shoot_speed = 0;
                        packet.unload_data(shoot_speed, 6);
                        if(shoot_speed > 0)
                            this->shoot_speed = shoot_speed;
                        if(this->debug)
                            RCLCPP_INFO(node_->get_logger(), "Shoot Speed: 【%d m/s】", shoot_speed);
                    }
                    //---- 三、获取我方颜色：color【6】 ----//
                    if (cmd == (unsigned char)frame_type::ChangeColor)
                    {
                        if(this->debug)
                            RCLCPP_INFO(node_->get_logger(), "[Color Get] RECV-package");
                        unsigned char color = 0x00;
                        packet.unload_data(color, 6);
                        if(color == 0xbb)
                        {
                            this->color = 0;
                            RCLCPP_INFO(node_->get_logger(), "Color: 【BLUE】");
                        }
                        else if(color == 0x11)
                        {
                            this->color = 1;
                            RCLCPP_INFO(node_->get_logger(), "Color: 【RED】");
                        }
                        else
                            RCLCPP_ERROR(node_->get_logger(), "ERROR Color package !!!");
                    }
                    //---- 四、获取当前姿态: tid【6-9】 yaw【10-13】 pitch【14-17】 roll【18-21】 time_stamp【22-26】 ----//
                    if (cmd == (unsigned char)frame_type::GimbalAngleControl)
                    {
                        if(this->debug)
                            RCLCPP_INFO(node_->get_logger(), "[Gimbel Angel Position] RECV-package");
                        uint32_t stm_tid = 0;
                        float yaw = 0.0, pitch = 0.0, roll = 0.0;
                        float time_stamp = 0.0;
                        rm_interfaces::msg::GyroAttitude Gyro_msg;
                        packet.unload_data(stm_tid, 6);
                        packet.unload_data(yaw, 10);
                        packet.unload_data(pitch, 14);
                        packet.unload_data(roll, 18);
                        packet.unload_data(time_stamp, 22);

                        Gyro_msg.tid = stm_tid;
                        Gyro_msg.yaw = yaw;
                        Gyro_msg.pitch = pitch;
                        Gyro_msg.roll = roll;
                        Gyro_msg.time_stamp = time_stamp;
                        gyro_attitude_pub_->publish(Gyro_msg);

                        if(this->debug)
                        {
                            // float Rpitch, Ryaw, Rroll;
                            // packet.unload_data(Rpitch, 3);
                            // packet.unload_data(Ryaw, 7);
                            // packet.unload_data(Rroll, 11);
                            RCLCPP_INFO(node_->get_logger(), "RECV-TID: '%d'", Gyro_msg.tid);
                            RCLCPP_INFO(node_->get_logger(), "RECV-PITCH: '%f'", Gyro_msg.yaw);
                            RCLCPP_INFO(node_->get_logger(), "RECV-YAW: '%f'", Gyro_msg.pitch);
                            RCLCPP_INFO(node_->get_logger(), "RECV-ROLL:'%f'", Gyro_msg.roll);
                            RCLCPP_INFO(node_->get_logger(), "RECV-TIME:'%f'", Gyro_msg.time_stamp);
                        }
                    }
                }
                // else
                // {
                //     RCLCPP_INFO(node_->get_logger(), "Serial Recv Fial!!!");
                // }
            }
        }
    }

    void SimpleRobotBaseNode::param_set_loop()
    {
        std::string serial_name_temp;
        int serial_bps_temp;
        bool serial_send;
        bool serial_recv;
        while (rclcpp::ok())
        {  
            //串口参数改变
            node_->get_parameter("serial_name",serial_name_temp);
            node_->get_parameter("serial_bps", serial_bps_temp);
            //节点收发功能开关
            node_->get_parameter("serial_send", serial_send);
            node_->get_parameter("serial_recv", serial_recv);
            
            auto transporter_temp = std::make_shared<UartTransporter>(serial_name_temp, serial_bps_temp, node_);

            if((serial_name_temp != this->serial_name))
            {
                this->serial_name = serial_name_temp;
                RCLCPP_INFO(node_->get_logger(), "serialName change to %s", serial_name_temp.c_str());   
                this->packet_tool_ = std::make_shared<FixedPacketTool<32>>(transporter_temp);
            }
            if((serial_bps_temp != this->serial_bps))
            {
                this->serial_bps = serial_bps_temp; 
                RCLCPP_INFO(node_->get_logger(), "serialBps change to %d", serial_bps_temp);  
                this->packet_tool_ = std::make_shared<FixedPacketTool<32>>(transporter_temp);
            }
            if((serial_send != this->SerialSend) || (serial_recv != this->SerialRecv))
            {
                this->SerialSend = serial_send;
                this->SerialRecv = serial_recv;
                RCLCPP_INFO(node_->get_logger(), "serial_send change to %d", serial_send);  
                RCLCPP_INFO(node_->get_logger(), "serial_recv change to %d", serial_recv); 
            }
        }
            
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_base::SimpleRobotBaseNode)
