#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/srv/get_mode.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

#include <memory>

class GetModeNode
{
public:
  explicit GetModeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

  // void get_mode_loop();
  void ModeSet(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
             std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rm_interfaces::srv::GetMode>::SharedPtr get_mode_cli_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  std::unique_ptr<std::thread> get_mode_thread_;
  int mode = 0;
};

GetModeNode::GetModeNode(const rclcpp::NodeOptions &options)
{
  node_ = std::make_shared<rclcpp::Node>("get_mode_client", options);
  // get_mode_cli_ = node_->create_client<rm_interfaces::srv::GetMode>("/recv/get_mode");

  // get_mode_thread_ = std::make_unique<std::thread>(&GetModeNode::get_mode_loop, this);
  set_mode_srv_ = node_->create_service<rm_interfaces::srv::SetMode>(
    "set_mode",
    // &SimpleRobotBaseNode::ModeGet);
    std::bind(&GetModeNode::ModeSet, this, std::placeholders::_1, std::placeholders::_2));
}

// void GetModeNode::get_mode_loop()
// {
//   while (!get_mode_cli_->wait_for_service(1s))
//   {
//     if (!rclcpp::ok())
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//       break;
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   auto get_mode_rqt_ = std::make_shared<rm_interfaces::srv::GetMode::Request>();
//   std::string n = "nashor";
//   get_mode_rqt_->node_type = n;

//   while (rclcpp::ok())
//   {
//     auto mode_get_result = get_mode_cli_->async_send_request(get_mode_rqt_);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Mode ：%d !", mode_get_result.get()->mode);
//     std::this_thread::sleep_for(std::chrono::microseconds(1000000));
//   }
// }

void GetModeNode::ModeSet(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                                  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
{
  response->success = true;
  int mode = request->mode;
  if (this->mode != mode)
  {
    RCLCPP_INFO(node_->get_logger(), "Now Mode : %x", mode);
    if (mode == 0xaa)
      RCLCPP_INFO(node_->get_logger(), "【Auto Aim】 Mode!");
    else if (mode == 0xbb)
      RCLCPP_INFO(node_->get_logger(), "【Smell Power Lune】 Mode!");
    else if (mode == 0xcc)
      RCLCPP_INFO(node_->get_logger(), "【Big Power Lune】 Mode!");
    else
      RCLCPP_INFO(node_->get_logger(), "【Normal】 Mode.");
  }
  this->mode = mode;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<GetModeNode>();
  rclcpp::spin(node_->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

/* ModeSet-service处理函数：模式切换



*/

// serial_state_thread_ = std::make_unique<std::thread>(&SimpleRobotBaseNode::serial_state_loop, this);

/* serial open/close
    void SimpleRobotBaseNode::serial_state_loop()
    {
        while (rclcpp::ok())
        {
            if(this->debug)
            {
                // transporter_->close();
                if(!this->packet_tool_->is_open())
                    RCLCPP_INFO(node_->get_logger(), "serial_name: %s Open Fail!!!",this->serial_name.c_str());
                else
                    RCLCPP_INFO(node_->get_logger(), "serial_name: %s Open Success!",this->serial_name.c_str());
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }
        }
    }*/