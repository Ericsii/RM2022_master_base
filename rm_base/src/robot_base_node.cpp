#include "rm_base/robot_base_node.hpp"
#include "rm_base/uart_transporter.hpp"
#include "rm_interfaces/qos_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;

namespace rm_base
{
    namespace frame_type
    {
        typedef enum : unsigned char
        {
            ChangeMode = 0xa1,
            GetShootSpeed = 0xb1,
            ChangeColor = 0xc1,
            GimbalAngleControl = 0xd1,
        } frame_type;
    }

    RobotBaseNode::RobotBaseNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("robot_base", options);
        this->node_name = node_->get_name();

        /*----------------------- Param ros参数 -----------------------*/
        //param参数：serial_name【串口名】
        node_->declare_parameter("serial_name", "/dev/ttyUSB0");
        node_->get_parameter("serial_name", this->serial_name);

        //param参数：serial_bps【串口波特率】
        node_->declare_parameter("serial_bps", 1152000);
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
        node_->declare_parameter("debug", false);
        this->debug = node_->get_parameter("debug").as_bool();

        /*----------------------- Topic ros话题 -----------------------*/
        if (custom_qos)
        {
            //串口传输Qos配置
            rclcpp::QoS cmd_gimbal_sub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
            rclcpp::QoS shoot_speed_pub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);

            //topic订阅：cmd_gimbal, 云台控制订阅，并将数据发送到串口
            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                "cmd_gimbal",
                cmd_gimbal_sub_qos_profile,
                std::bind(&RobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1));
            //topic发布：shoot_speed, 真实射速发布
            shoot_speed_pub_ = node_->create_publisher<rm_interfaces::msg::ShootSpeed>(
                "shoot_speed",
                shoot_speed_pub_qos_profile);
        }
        else
        {
            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                "cmd_gimbal",
                10,
                std::bind(&RobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1));
            shoot_speed_pub_ = node_->create_publisher<rm_interfaces::msg::ShootSpeed>(
                "shoot_speed",
                10);
        }

        /*----------------------- Service ros服务 -----------------------*/
        //service服务端：获取模式
        get_mode_srv_ = node_->create_service<rm_interfaces::srv::GetMode>(
            "get_mode",
            std::bind(&RobotBaseNode::ModeGet, this, std::placeholders::_1, std::placeholders::_2));

        //service服务端：获取颜色
        get_color_srv_ = node_->create_service<rm_interfaces::srv::GetColor>(
            "get_color",
            std::bind(&RobotBaseNode::ColorGet, this, std::placeholders::_1, std::placeholders::_2));

        //client客户端：设置击打模式
        set_autoaim_mode_cli_ = node_->create_client<rm_interfaces::srv::SetMode>(
            "auto_aim/set_mode");
        set_nahsor_mode_cli_ = node_->create_client<rm_interfaces::srv::SetMode>(
            "nahsor/set_mode");

#ifdef RM_DEBUG_MODE
        if (this->debug)
            RCLCPP_INFO(node_->get_logger(), "Serial:%s(%d) Init ", this->serial_name.c_str(), this->serial_bps);
        if (this->SerialRecv)
            RCLCPP_INFO(node_->get_logger(), "SerialRecv ON!");
        if (this->SerialSend)
            RCLCPP_INFO(node_->get_logger(), "SerialSend ON!");
        if (custom_qos)
            RCLCPP_INFO(node_->get_logger(), "Qos ON!");
        RCLCPP_INFO(node_->get_logger(), "Service Mode/Color/Shoot-Speed start!");
#endif

        /*----------------------- Thread 线程 -----------------------*/
        //thread线程：串口数据接收
        listen_thread_ = std::make_unique<std::thread>(&RobotBaseNode::listen_loop, this);
    }

    //topic订阅返回函数：发送串口数据
    void RobotBaseNode::gimbal_cmd_cb(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
    {
        if (this->SerialSend)
        {
            if ((this->tid >= (2147483647 - 100)))
                this->tid = 0;
            this->tid++;
            FixedPacket<32> packet;
            packet.load_data<uint32_t>(this->tid, 1);

            // ----将数据导入数据包中----//
            // ----上位机->下位机----/
            /** 哨兵、步兵、英雄
             * sentry、infantry、hero
             * 1-4  tid 包编号
             * 5    type  巡逻、瞄准、开枪
             * 6    shoot
             * 5-8  yaw
             * 9-12 pitch */
            //（【5 type】0x2a自瞄,0x3a巡逻,0x4a遥控器 【6 shoot】0x4b发射）
            packet.load_data<unsigned char>(msg->type, 5);
            packet.load_data<unsigned char>(msg->shoot, 6);
            packet.load_data<float>(msg->position.yaw, 7);
            packet.load_data<float>(msg->position.pitch, 11);

#ifdef RM_DEBUG_MODE
            if (this->debug)
            {
                this->time_send = rclcpp::Clock().now();
                //----第一次发送----//
                if (this->tid == 1)
                    RCLCPP_INFO(node_->get_logger(), "Serial:%s(%d) Send Init ", this->serial_name.c_str(), this->serial_bps);
                else
                    RCLCPP_INFO(node_->get_logger(), "\nSEND packet [%d]", this->tid);
                uint32_t id = 0;
                if (packet.unload_data(id, 1))
                    RCLCPP_INFO(node_->get_logger(), "SEND-ID: '%d'", id);

                RCLCPP_INFO(node_->get_logger(), "SEND-NAMESPACE: %s", node_->get_namespace());

                float pitch = 0;
                float yaw = 0;
                packet.unload_data<float>(yaw, 7);
                packet.unload_data<float>(pitch, 11);
                RCLCPP_INFO(node_->get_logger(), "pitch [%f]", pitch);
                RCLCPP_INFO(node_->get_logger(), "yaw [%f]", yaw);
                RCLCPP_INFO(node_->get_logger(), "type [%d]", msg->type);
                RCLCPP_INFO(node_->get_logger(), "shoot [%d]", msg->shoot);
            }
#endif
            // 设置校验位字节（check_byte）为BCC校验码
            packet.set_check_byte();
            // 通过串口发送包到下位机
            this->packet_tool_->send_packet(packet);
            //清理缓存
            packet.clear();
        }
    }

    // service处理函数：模式切换
    void RobotBaseNode::ModeGet(const std::shared_ptr<rm_interfaces::srv::GetMode::Request> request,
                                std::shared_ptr<rm_interfaces::srv::GetMode::Response> response)
    {
        response->mode = this->mode;
        std::string node_type = request->node_type;
#ifdef RM_DEBUG_MODE
        if (this->debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Get Mode Client: %s", node_type.c_str());
            if (this->mode == 0xaa)
                RCLCPP_INFO(node_->get_logger(), "【Auto Aim】 Mode!");
            else if (this->mode == 0xbb)
                RCLCPP_INFO(node_->get_logger(), "【Smell Power Lune】 Mode!");
            else if (this->mode == 0xcc)
                RCLCPP_INFO(node_->get_logger(), "【Big Power Lune】 Mode!");
            else
                RCLCPP_INFO(node_->get_logger(), "【Normal】 Mode.");
        }
#endif
    }

    void RobotBaseNode::ColorGet(const std::shared_ptr<rm_interfaces::srv::GetColor::Request> request,
                                 std::shared_ptr<rm_interfaces::srv::GetColor::Response> response)
    {
        response->color = this->color;
        std::string node_type = request->node_type;
#ifdef RM_DEBUG_MODE
        if (this->debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Get Color Client: %s", node_type.c_str());
            if (this->color == 0)
                RCLCPP_INFO(node_->get_logger(), "Ours Color is 【BLUE】!");
            else
                RCLCPP_INFO(node_->get_logger(), "Ours Color is 【RED】!");
        }
#endif
    }

    // 接收串口数据
    void RobotBaseNode::listen_loop()
    {
        FixedPacket<32> packet;
        uint32_t recv_tid = 0;
        while (rclcpp::ok())
        {
            this->SerialRecv = node_->get_parameter("serial_recv").as_bool();
            if (this->SerialRecv)
            {
                if (this->packet_tool_->recv_packet(packet))
                {
                    //----接收包编号----//
                    /*recv_tid【1-4】:包编号位 0x00000000-0xffffffff*/
                    packet.unload_data(recv_tid, 1);

                    //------接收包种类------//
                    /*cmd【5】:帧种类位 0xa1云台控制 0xb1射速改变 0xc1模式改变*/
                    unsigned char cmd;
                    packet.unload_data(cmd, 5);
#ifdef RM_DEBUG_MODE
                    if (this->debug)
                    {
                        RCLCPP_INFO(node_->get_logger(), "\nRECV packet [%d]", recv_tid);
                        RCLCPP_INFO(node_->get_logger(), "RECV-cmd: '%x'", cmd);
                    }
#endif
                    //---- 一、模式切换: mode【6】 ----//
                    if (cmd == (unsigned char)frame_type::ChangeMode)
                    {
                        /*mode【6】:模式切换位 0x01自瞄 0xbb小能量机关 0xcc大能量机关 0xdd测试 0x00正常模式*/
                        unsigned char mode = 0x00;
                        packet.unload_data(mode, 6);

                        if(mode == 0xdd) mode = 0x01;

                        if ((mode == 0x01) || (mode == 0x02) || (mode == 0xbb) || (mode == 0xcc) || (mode == 0xdd))
                        {
                            this->SerialSend = true; //开启发送部分，开始向下位机发送数据
                        }
                        else if (mode == 0x00)
                        {
                            this->SerialSend = false; //关闭发送部分，正常模式
                        }
                        else
                        {
                            this->SerialSend = false;
                            RCLCPP_ERROR(node_->get_logger(), "【MODE %x ERROR】!!!", mode);
                        }

                        auto set_mode_rqt_ = std::make_shared<rm_interfaces::srv::SetMode::Request>();
                        //this->mode为上一状态模式
                        if (mode != this->mode)
                        {
                            mode_change_id = 0;
                            if (mode == 0x01 || mode == 0x02 || (mode == 0xdd))
                            {
                                set_mode_rqt_->mode = mode;
                                while (!set_autoaim_mode_cli_->wait_for_service(1s)) 
                                {
                                    RCLCPP_ERROR(node_->get_logger(), "Auto Aim Mode Change Service Not Available!!!");
                                    if(mode_change_id++ >1000)
                                    {
                                        mode_change_id = 0;
                                        break;
                                    }
                                }
                                if (set_autoaim_mode_cli_->wait_for_service(1s))
                                {
                                    auto mode_set_result = set_autoaim_mode_cli_->async_send_request(set_mode_rqt_);
                                    while (!mode_set_result.get()->success)
                                    {
                                        RCLCPP_ERROR(node_->get_logger(), "Auto Aim Mode Change Fail!!!");
                                        mode_set_result = set_autoaim_mode_cli_->async_send_request(set_mode_rqt_);
                                    }
                                }
                            }
                            else if (mode == 0xbb || mode == 0xcc)
                            {
                                set_mode_rqt_->mode = mode;
                                while (!set_nahsor_mode_cli_->wait_for_service(1s))
                                {
                                    RCLCPP_ERROR(node_->get_logger(), "Nahsor Mode Change Service Not Available!!!");
                                    if (mode_change_id++ > 1000)
                                    {
                                        mode_change_id = 0;
                                        break;
                                    }
                                }
                                if (set_nahsor_mode_cli_->wait_for_service(1s))
                                {
                                    auto mode_set_result = set_nahsor_mode_cli_->async_send_request(set_mode_rqt_);
                                    while (!mode_set_result.get()->success)
                                    {
                                        RCLCPP_ERROR(node_->get_logger(), "Nahsor Mode Change Fail!!!");
                                        mode_set_result = set_nahsor_mode_cli_->async_send_request(set_mode_rqt_);
                                    }
                                }
                            }
                        }
                        this->mode = mode;
                        
#ifdef RM_DEBUG_MODE
                        if (this->debug)
                        {
                            RCLCPP_INFO(node_->get_logger(), "RECV package type [Mode Change]");
                            RCLCPP_INFO(node_->get_logger(), "RECV-mode: '%x'", this->mode);
                            if (mode == 0x01 || mode == 0x02)
                            {
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Auto Aim】");
                            }
                            else if (mode == 0xbb)
                            {
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Smell Nashor】");
                            }
                            else if (mode == 0xcc)
                            {
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Big Nashor】");
                            }
                            else if (mode == 0x00)
                            {
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Normal】");
                            }
                            else
                            {
                                RCLCPP_ERROR(node_->get_logger(), "【MODE】ERROR!!!");
                            }
                        }
#endif
                    }
                    //---- 二、获取当前真实射速: shoot_speed【6-9】 ----//
                    if (cmd == (unsigned char)frame_type::GetShootSpeed)
                    {
                        if (recv_tid <= this->last_tid)
                            continue;

                        float shoot_speed = 0;
                        float r = 0.5; //低级滤波参数
                        rm_interfaces::msg::ShootSpeed Shoot_Speed_msg;
                        packet.unload_data(shoot_speed, 6);
                        if (shoot_speed > 0)
                        {
                            Shoot_Speed_msg.shoot_speed = r * this->last_shoot_speed + (1 - r) * shoot_speed;
                            shoot_speed_pub_->publish(Shoot_Speed_msg);
                        }
                        else
                            RCLCPP_ERROR(node_->get_logger(), "【SHOOT-SPEED】ERROR!!!");

#ifdef RM_DEBUG_MODE
                        if (this->debug)
                        {
                            RCLCPP_INFO(node_->get_logger(), "RECV package type [Shoot-speed Set]");
                            RCLCPP_INFO(node_->get_logger(), "Real Shoot Speed: 【%f m/s】", shoot_speed);
                            RCLCPP_INFO(node_->get_logger(), "Last Shoot Speed: 【%f m/s】", this->last_shoot_speed);
                            RCLCPP_INFO(node_->get_logger(), "Pub Shoot Speed: 【%f m/s】", Shoot_Speed_msg.shoot_speed);
                        }
#endif
                        this->last_shoot_speed = shoot_speed;
                    }
                    //---- 三、获取我方颜色：color【6】 ----//
                    if (cmd == (unsigned char)frame_type::ChangeColor)
                    {
                        unsigned char color = 0x00;
                        packet.unload_data(color, 6);
                        if (color == 0xbb)
                        {
                            this->color = 0;
                        }
                        else if (color == 0x11)
                        {
                            this->color = 1;
                        }
                        else
                            RCLCPP_ERROR(node_->get_logger(), "【Color】ERROR !!!");
#ifdef RM_DEBUG_MODE
                        if (this->debug)
                        {
                            RCLCPP_INFO(node_->get_logger(), "RECV package type [Color Get]");
                            if (color == 0xbb)
                            {
                                RCLCPP_INFO(node_->get_logger(), "Color: 【BLUE】");
                            }
                            else if (color == 0x11)
                            {
                                RCLCPP_INFO(node_->get_logger(), "Color: 【RED】");
                            }
                            else
                                RCLCPP_ERROR(node_->get_logger(), "ERROR Color package !!!");
                        }
#endif
                    }
                    this->last_tid = recv_tid;
                }
            }
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_base::RobotBaseNode)
