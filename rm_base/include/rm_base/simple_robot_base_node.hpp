#ifndef RM_BASE__SIMPLE_ROBOT_BASE_NODE_HPP_
#define RM_BASE__SIMPLE_ROBOT_BASE_NODE_HPP_

#include <thread>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rm_base/transporter_interface.hpp"
#include "rm_base/fixed_packet_tool.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"

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

        void listen_loop();
        void param_set_loop();

    private:
        rclcpp::Node::SharedPtr node_;
        //线程：串口接收线程、参数设置线程
        std::unique_ptr<std::thread> listen_thread_;  
        std::unique_ptr<std::thread> param_set_thread_;
        // 接口工具
        TransporterInterface::SharedPtr transporter_;
        FixedPacketTool<16>::SharedPtr packet_tool_;
        //订阅
        rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_gimbal_sub_;
        void gimbal_cmd_cb(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);
        //串口配置
        std::string serial_name;
        bool SerialSend = false;
        bool SerialRecv = true;
    };
}

#endif
