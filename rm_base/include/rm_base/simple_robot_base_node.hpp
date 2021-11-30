#ifndef RM_BASE__SIMPLE_ROBOT_BASE_NODE_HPP_
#define RM_BASE__SIMPLE_ROBOT_BASE_NODE_HPP_

#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rm_base/transporter_interface.hpp"
#include "rm_base/fixed_packet_tool.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/shoot_speed.hpp"
#include "rm_interfaces/srv/get_mode.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_interfaces/srv/get_color.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <math.h>

namespace rm_base
{
    class SimpleRobotBaseNode
    {
    public:
        explicit SimpleRobotBaseNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

        //线程执行函数
        void listen_loop();
        void param_set_loop();

        /**
         * @brief Service服务端返回处理函数
         * 
         * @param request 
         * @param response 
         */
        void ModeGet(const std::shared_ptr<rm_interfaces::srv::GetMode::Request> request,
                                        std::shared_ptr<rm_interfaces::srv::GetMode::Response> response);
        void ColorGet(const std::shared_ptr<rm_interfaces::srv::GetColor::Request> request,
                                        std::shared_ptr<rm_interfaces::srv::GetColor::Response> response);


    private:
        /**
         * @brief ros结点
         */
        rclcpp::Node::SharedPtr node_;

        /**
         * @brief 线程
         */
        std::unique_ptr<std::thread> listen_thread_;
        std::unique_ptr<std::thread> param_set_thread_;
        std::unique_ptr<std::thread> serial_state_thread_;

        /**
         * @brief 接口工具
         */
        TransporterInterface::SharedPtr transporter_;
        FixedPacketTool<32>::SharedPtr packet_tool_;

        /**
         * @brief 【ROS2】订阅-subscription、发布-publisher、服务端-service
         */
        rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_gimbal_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;
        rclcpp::Publisher<rm_interfaces::msg::ShootSpeed>::SharedPtr shoot_speed_pub_;
        rclcpp::Service<rm_interfaces::srv::GetMode>::SharedPtr get_mode_srv_;
        rclcpp::Service<rm_interfaces::srv::GetColor>::SharedPtr get_color_srv_;
        rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr set_mode_cli_;
        /**
         * @brief 订阅处理函数
         * 
         * @param msg 
         */
        void gimbal_cmd_cb(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);
        
        /**
         * @brief ros参数--全局变量
         */
        std::string serial_name;
        int serial_bps;
        bool SerialSend;
        bool SerialRecv;
        bool debug;

        /**
         * @brief 其他变量
         */
        uint32_t tid = 0;                   //包编号
        uint32_t last_tid = 0;
        int mode = 0;                       //模式：0-正常，1-自瞄，2-小符，3-大符
        int color = 1;                      //颜色：0-blue，1-red
        rclcpp::Time time_send;
        rclcpp::Time time_recv;
        std::string node_name;
        double max_time = 0;
        double min_time = 10000.0;
        float last_shoot_speed = 0;
    };
}

#endif
