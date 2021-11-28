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
#include <vector>

namespace rm_base
{
    namespace frame_type
    {
        typedef enum : unsigned char
        {
            ChangeMode = 0xa1,
            GetShootSpeed= 0xb1,
            ChangeColor = 0xc1,
            GimbalAngleControl= 0xd1
        } frame_type;
    }

    SimpleRobotBaseNode::SimpleRobotBaseNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("simple_robot_base", options);
        this->node_name = node_->get_name();

        /*----------------------- Param ros参数 -----------------------*/
        //param参数：serial_name【串口名】
        node_->declare_parameter("serial_name", "/dev/ttyUSB0");
        node_->get_parameter("serial_name",this->serial_name);

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
            rclcpp::QoS gyro_quaternions_pub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
            rclcpp::QoS shoot_speed_pub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);

            //topic订阅：cmd_gimbal, 云台控制订阅，并将数据发送到串口
            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                this->node_name + "/cmd_gimbal",
                cmd_gimbal_sub_qos_profile,
                std::bind(&SimpleRobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1)
                );
            //topic发布：gyro_quaternions, 陀螺仪姿态发布
            gyro_quaternions_pub_ = node_->create_publisher<rm_interfaces::msg::GyroQuaternions>(
                this->node_name + "/gyro_quaternions",
                gyro_quaternions_pub_qos_profile);
            //topic发布：shoot_speed, 真实射速发布
            shoot_speed_pub_ = node_->create_publisher<rm_interfaces::msg::ShootSpeed>(
                this->node_name + "/shoot_speed",
                shoot_speed_pub_qos_profile);
        }
        else
        {
            cmd_gimbal_sub_ = node_->create_subscription<rm_interfaces::msg::GimbalCmd>(
                this->node_name + "/cmd_gimbal",
                10,
                std::bind(&SimpleRobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1)
                );
            gyro_quaternions_pub_ = node_->create_publisher<rm_interfaces::msg::GyroQuaternions>(
                this->node_name + "/gyro_quaternions",
                10);
            shoot_speed_pub_ = node_->create_publisher<rm_interfaces::msg::ShootSpeed>(
                this->node_name + "/shoot_speed",
                10);
        }  

        /*----------------------- Service ros服务 -----------------------*/
        //service服务端：获取模式
        get_mode_srv_ = node_->create_service<rm_interfaces::srv::GetMode>(
            this->node_name + "/get_mode", 
            // &SimpleRobotBaseNode::ModeGet);
            std::bind(&SimpleRobotBaseNode::ModeGet, this, std::placeholders::_1, std::placeholders::_2)
            );

        //service服务端：获取颜色
        get_color_srv_ = node_->create_service<rm_interfaces::srv::GetColor>(
            this->node_name + "/get_color",
            std::bind(&SimpleRobotBaseNode::ColorGet, this, std::placeholders::_1, std::placeholders::_2)
            );

#ifdef DEBUG_MODE
        if(this->debug)
            RCLCPP_INFO(node_->get_logger(), "Serial:%s(%d) Init ", this->serial_name.c_str(), this->serial_bps);
            if(this->SerialRecv)
                RCLCPP_INFO(node_->get_logger(), "SerialRecv ON!");
            if(this->SerialSend)
                RCLCPP_INFO(node_->get_logger(), "SerialSend ON!");
            if(custom_qos)
                RCLCPP_INFO(node_->get_logger(), "Qos ON!");
            RCLCPP_INFO(node_->get_logger(), "Service Mode/Color/Shoot-Speed start!");
#endif

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
            if((this->tid >= (2147483647-100)))
                this->tid = 1;
            this->tid++;
            //----将数据导入数据包中----//
            //----上位机->下位机----/

            FixedPacket<32> packet;
            packet.load_data<uint32_t>(this->tid, 1);

            if(this->node_name == "sentry")
            {
                /** 哨兵 sentry
                 * 1-4  tid 包编号
                 * 5    cmd 
                 * 6    mode
                 * 5-8  yaw 
                 * 9-12 pitch */
                packet.load_data<unsigned char>(0x05, 5);
                packet.load_data<unsigned char>(0x05, 6);
                float yaw = 0.0, pitch = 260.0 + float(tid%60);
                if ((tid % 100) >= 50 )
                    yaw = 100 - tid%100;
                else
                    yaw = tid%100;
                
                if ((tid % 80) >= 40 )
                    pitch = 260.0+ float(80 - tid%80);
                else
                    pitch = 260.0 + float(tid%80);

                packet.load_data<float>(yaw, 7);
                packet.load_data<float>(pitch, 11);
            }
            else
            {
                /** 步兵 infantry、英雄 hero
                 * 1-4  tid 包编号
                 * 5-8  yaw 
                 * 9-12 pitch */
                packet.load_data<float>(msg->position.yaw, 7);
                packet.load_data<float>(msg->position.pitch, 11);
            }
            /** RMUA
            packet.load_data<float>(msg->velocity.yaw, 13);
            packet.load_data<float>(msg->velocity.pitch, 17);
            packet.load_data<int>(shoot, 21);
            **/

#ifdef DEBUG_MODE
            if(this->debug)
            {
                this->time_send = rclcpp::Clock().now();
                //----第一次发送----//
                if (this->tid == 1)
                    RCLCPP_INFO(node_->get_logger(), "Serial:%s(%d) Send Init ", this->serial_name.c_str(), this->serial_bps);
                else
                    RCLCPP_INFO(node_->get_logger(), "\nSEND packet [%d]", this->tid);

                // packet.load_data<unsigned char>(frame_type::ChangeMode, 5);
                // packet.load_data<unsigned char>(0xbb, 6);
                // packet.load_data<unsigned char>(0x0d, 7);
                // packet.load_data<unsigned char>(0xff, 8);

                // packet.load_data<unsigned char>(frame_type::GetShootSpeed, 5);
                // packet.load_data<int>(18, 6);

                // packet.load_data<unsigned char>(frame_type::ChangeColor, 5);
                // packet.load_data<unsigned char>(0xbb, 6);

                packet.load_data<unsigned char>(frame_type::GimbalAngleControl, 5);
                packet.load_data<float>(1.0, 6);
                packet.load_data<float>(2.0, 10);
                packet.load_data<float>(3.0, 14);
                packet.load_data<float>(3.0, 18);
                packet.load_data<double>(this->time_send.seconds(), 22);

                // unsigned char bcc = 0x11, c=0x00;
                uint32_t id = 0;
                // float Spitch, Syaw;
                // packet.unload_data(Spitch, 7);
                // packet.unload_data(Syaw, 11);
                // RCLCPP_INFO(node_->get_logger(), "SEND-PITCH packet '%f'",Spitch);
                // RCLCPP_INFO(node_->get_logger(), "SEND-YAW packet: '%f'", Syaw);
                if(packet.unload_data(id, 1))
                    RCLCPP_INFO(node_->get_logger(), "SEND-ID: '%d'",  id);
                double timex;
                packet.unload_data<double>(timex, 22);
                RCLCPP_INFO(node_->get_logger(), "SEND-time: '%f'",  timex);
                // if(packet.unload_data(bcc, 30))
                //     RCLCPP_INFO(node_->get_logger(), "SEND-BCC: '%x'",  bcc);
            }      
#endif            

            // 设置校验位字节（check_byte）为BCC校验码
            packet.set_check_byte();       
            // 通过串口发送包到下位机
            this->packet_tool_->send_packet(packet);
            
#ifdef DEBUG_MODE
            if (this->debug)
            {
                bool send_ack = this->packet_tool_->send_packet(packet);
                if (!send_ack)
                    RCLCPP_ERROR(node_->get_logger(), "Serial Send Fail!!!"); 
                double time1, time2;
                time1 = this->time_send.seconds();
                time2 = rclcpp::Clock().now().seconds();
                RCLCPP_INFO(node_->get_logger(),"send[%d]: %f",this->tid,(time2-time1)); 
                RCLCPP_INFO(node_->get_logger(),"start[%d]: %f",this->tid,time2); 
            }
#endif

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
#ifdef DEBUG_MODE
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
#endif
    }

    void SimpleRobotBaseNode::ColorGet(const std::shared_ptr<rm_interfaces::srv::GetColor::Request> request, 
                                            std::shared_ptr<rm_interfaces::srv::GetColor::Response> response)
    {
        response->color = this->color;
        std::string node_type = request->node_type;
#ifdef DEBUG_MODE
        if(this->debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Get Color Client: %s", node_type.c_str());
            if(this->color == 0)
                RCLCPP_INFO(node_->get_logger(), "Ours Color is 【BLUE】!");
            else
                RCLCPP_INFO(node_->get_logger(), "Ours Color is 【RED】!");
        }
#endif
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
                    if(recv_tid <= this->last_tid)
                        continue;
                        
                    //------接收包种类------//
                    /*cmd【5】:帧种类位 0xa1云台控制 0xb1射速改变 0xc1模式改变*/
                    unsigned char cmd;             
                    packet.unload_data(cmd, 5);
#ifdef DEBUG_MODE
                    if (this->debug)
                    {
                        RCLCPP_INFO(node_->get_logger(), "RECV-cmd: '%x'", cmd);
                        this->time_recv = rclcpp::Clock().now();
                        RCLCPP_INFO(node_->get_logger(), "\nRECV packet [%d]", recv_tid);
                        double timer = rclcpp::Clock().now().seconds();
                        RCLCPP_INFO(node_->get_logger(),"recv[%d]: %f",recv_tid,timer); 
                    }
#endif      
                    
                    //---- 一、模式切换: mode【6】 ----//
                    if (cmd == (unsigned char)frame_type::ChangeMode)
                    {
                        /*mode【6】:模式切换位 0xaa自瞄 0xbb小能量机关 0xcc大能量机关*/
                        unsigned char mode = 0x00;
                        packet.unload_data(mode, 6);
#ifdef DEBUG_MODE
                        if(this->debug)
                        {
                            RCLCPP_INFO(node_->get_logger(), "RECV package type [Mode Change]");
                            RCLCPP_INFO(node_->get_logger(), "RECV-mode: '%x'", mode); 
                            if (mode == 0xaa){
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Auto Aim】");
                            } else if (mode == 0xbb){
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Smell Nashor】");
                            } else if (mode == 0xcc){
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Big Nashor】");
                            } else if (mode == 0xee){
                                RCLCPP_INFO(node_->get_logger(), "Mode：【Normal】");
                            } else {
                                RCLCPP_ERROR(node_->get_logger(), "【MODE】ERROR!!!");
                            }
                        }
#endif                            
                        if ((mode == 0xaa)||(mode == 0xbb)||(mode == 0xcc)) {
                            this->SerialSend = true;    //开启发送部分，开始向下位机发送数据 
                        }    
                        else if (mode == 0xee){
                            this->SerialSend = false;   //关闭发送部分，正常模式
                        }
                        else
                            RCLCPP_ERROR(node_->get_logger(), "【MODE】ERROR!!!");
                         
                        if (mode == 0xaa){
                            this->mode = 1;
                        } else if (mode == 0xbb){
                            this->mode = 2;
                        } else if (mode == 0xcc){
                            this->mode = 3;
                        } else {
                            this->mode = 0;
                            this->SerialSend = false;
                        }
                    }
                    //---- 二、获取当前真实射速: shoot_speed【6-9】 ----//
                    if (cmd == (unsigned char)frame_type::GetShootSpeed)
                    {
                        int shoot_speed = 0;
                        rm_interfaces::msg::ShootSpeed Shoot_speed_msg;
                        packet.unload_data(shoot_speed, 6);
                        if(shoot_speed > 0)
                        {
                            Shoot_speed_msg.shoot_speed = shoot_speed;
                            shoot_speed_pub_->publish(Shoot_speed_msg);
                        }
                        else
                            RCLCPP_ERROR(node_->get_logger(), "【SHOOT-SPEED】ERROR!!!");
                        
                        
#ifdef DEBUG_MODE
                        if(this->debug)
                        {
                            RCLCPP_INFO(node_->get_logger(), "RECV package type [Shoot-speed Get]");
                            RCLCPP_INFO(node_->get_logger(), "Shoot Speed: 【%d m/s】", shoot_speed);
                        }
#endif
                    }
                    //---- 三、获取我方颜色：color【6】 ----//
                    if (cmd == (unsigned char)frame_type::ChangeColor)
                    {
                        unsigned char color = 0x00;
                        packet.unload_data(color, 6);
                        if(color == 0xbb)
                        {
                            this->color = 0;
                        }
                        else if(color == 0x11)
                        {
                            this->color = 1;
                        }
                        else
                            RCLCPP_ERROR(node_->get_logger(), "【Color】ERROR !!!");
#ifdef DEBUG_MODE
                        if(this->debug)
                        {
                            RCLCPP_INFO(node_->get_logger(), "RECV package type [Color Get]");
                            if(color == 0xbb){
                                RCLCPP_INFO(node_->get_logger(), "Color: 【BLUE】");
                            } else if(color == 0x11){
                                RCLCPP_INFO(node_->get_logger(), "Color: 【RED】");
                            } else
                                RCLCPP_ERROR(node_->get_logger(), "ERROR Color package !!!");
                        }
#endif  
                    }
                    //---- 四、获取当前姿态:  Q1-Q4【6-9】【10-13】【14-17】【18-21】 time_stamp【22-29】 ----//
                    if (cmd == (unsigned char)frame_type::GimbalAngleControl)
                    {
                        std::array<float, 4UL> Q;
                        double time_stamp = 0.0;
                        rm_interfaces::msg::GyroQuaternions Gyro_msg;
                        packet.unload_data(Q, 6);
                        packet.unload_data(time_stamp, 22);

                        Gyro_msg.tid = recv_tid;
                        Gyro_msg.q = Q;
                        Gyro_msg.time_stamp = time_stamp;
                        gyro_quaternions_pub_->publish(Gyro_msg);
#ifdef DEBUG_MODE
                        if(this->debug)
                        {
                            // float yaw = 0.0, pitch = 0.0, roll = 0.0;
                            // rm_interfaces::msg::Gyroquaternions Gyro_msg;
                            // packet.unload_data(yaw, 6);
                            // packet.unload_data(pitch, 10);
                            // packet.unload_data(roll, 14);
                            // Gyro_msg.yaw = yaw;
                            // Gyro_msg.pitch = pitch;
                            // Gyro_msg.roll = roll;
                            RCLCPP_INFO(node_->get_logger(), "RECV package type [Gimbel Angel Position]");
                            RCLCPP_INFO(node_->get_logger(), "RECV-TID: '%d'", Gyro_msg.tid);
                            RCLCPP_INFO(node_->get_logger(), "RECV-GyroQuaternions: (%f, %f, %f, %f)", Gyro_msg.q[0], Gyro_msg.q[1], Gyro_msg.q[2], Gyro_msg.q[3]);
                            RCLCPP_INFO(node_->get_logger(), "RECV-TIME:'%f'", Gyro_msg.time_stamp);
                        }
#endif
                    }

#ifdef DEBUG_MODE
                    if (this->debug)
                    {
                        double time1, time2, time3;
                        time1 = this->time_recv.seconds();
                        time2 = rclcpp::Clock().now().seconds();
                        time3 = this->time_send.seconds();
                        // if (recv_tid == this->tid)
                        // packet.unload_data<double>(time3, 18);
                        // RCLCPP_INFO(node_->get_logger(),"\nsend-time %f, \nrecv-time %f",time3, time2);
                        RCLCPP_INFO(node_->get_logger(),"all[%d]: %f",recv_tid,(time2-time3));
 
                        // time3 = this->time_send.seconds();
                        RCLCPP_INFO(node_->get_logger(),"recv[%d]: %f",recv_tid,(time2-time1)); 

                        if (this->max_time < (time2-time3) )
                            this->max_time = (time2-time3);
                        if (this->min_time > (time2-time3) )
                            this->min_time = (time2-time3);

                        RCLCPP_INFO(node_->get_logger(),"all[%d]: %f",recv_tid,(time2-time3));
                        RCLCPP_INFO(node_->get_logger(),"max: %f", this->max_time);
                        RCLCPP_INFO(node_->get_logger(),"min: %f", this->min_time);
                    }
#endif
                    this->last_tid = recv_tid;
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
