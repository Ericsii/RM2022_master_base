#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/gyro_quaternions.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("subscriber")
    {
        gyro_quaternions_sub_= this->create_subscription<rm_interfaces::msg::GyroQuaternions>(
                "recv/gyro_quaternions",
                10,
                std::bind(&MinimalSubscriber::gyro_quaternions_cb, this, std::placeholders::_1)
            );
    }

  private:
    void gyro_quaternions_cb(const rm_interfaces::msg::GyroQuaternions::SharedPtr msg) 
    {
      RCLCPP_INFO(this->get_logger(), "RECV package type [Gimbel Angel Position]");
      RCLCPP_INFO(this->get_logger(), "RECV-TID: '%d'", msg->tid);
      RCLCPP_INFO(this->get_logger(), "RECV-GyroQuaternions: (%f, %f, %f, %f)", msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
      RCLCPP_INFO(this->get_logger(), "RECV-TIME:'%f'", msg->time_stamp);
    }
    rclcpp::Subscription<rm_interfaces::msg::GyroQuaternions>::SharedPtr gyro_quaternions_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
