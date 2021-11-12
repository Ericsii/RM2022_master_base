#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/gyro_attitude.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("subscriber")
    {
        gyro_attitude_sub_= this->create_subscription<rm_interfaces::msg::GyroAttitude>(
                "gyro_attitude",
                10,
                std::bind(&MinimalSubscriber::gyro_attitude_cb, this, std::placeholders::_1)
            );
    }

  private:
    void gyro_attitude_cb(const rm_interfaces::msg::GyroAttitude::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Gyro yaw: %f", msg->yaw);
        RCLCPP_INFO(this->get_logger(), "Gyro pitch: %f", msg->pitch);
        RCLCPP_INFO(this->get_logger(), "Gyro roll: %f", msg->roll);
        RCLCPP_INFO(this->get_logger(), "Gyro tid: %d", msg->tid);
        RCLCPP_INFO(this->get_logger(), "Gyro time: %f", msg->time_stamp);
    }
    rclcpp::Subscription<rm_interfaces::msg::GyroAttitude>::SharedPtr gyro_attitude_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
