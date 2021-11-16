#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/srv/get_mode.hpp"
#include "rm_interfaces/msg/gyro_attitude.hpp"
#include <chrono>
using namespace std::chrono_literals;

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("get_mode_client");

  rclcpp::Client<rm_interfaces::srv::GetMode>::SharedPtr get_mode_cli_ = 
    node_->create_client<rm_interfaces::srv::GetMode>("/recv/get_mode");

  auto get_mode_rqt_ = std::shared_ptr<rm_interfaces::srv::GetMode::Request>();
  // get_mode_rqt_->node_type = "nashor";
  // get_mode_rqt_->node_type = atoll(argv[1]);
  get_mode_rqt_->node_type = argv[1];

  while (!get_mode_cli_->wait_for_service(1s))
  {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto mode_get_result = get_mode_cli_->async_send_request(get_mode_rqt_);

  if (rclcpp::spin_until_future_complete(node_, mode_get_result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Mode ：%d Success!", mode_get_result.get()->mode);
  } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_mode");
  }

  rclcpp::shutdown();
  return 0;
}


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