#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/gyro_quaternions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("subscriber")
    {
        // gyro_quaternions_sub_= this->create_subscription<rm_interfaces::msg::GyroQuaternions>(
        //         "recv/gyro_quaternions",
        //         10,
        //         std::bind(&MinimalSubscriber::gyro_quaternions_cb, this, std::placeholders::_1)
        //     );

        pose_stamped_sub_= this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "sentry/pose_stamped",
                10,
                std::bind(&MinimalSubscriber::pose_stamped_cb, this, std::placeholders::_1)
            );
    }

  private:
    // void gyro_quaternions_cb(const rm_interfaces::msg::GyroQuaternions::SharedPtr msg) 
    // {
    //   RCLCPP_INFO(this->get_logger(), "RECV package type [Gimbel Angel Position]");
    //   RCLCPP_INFO(this->get_logger(), "RECV-TID: '%d'", msg->tid);
    //   RCLCPP_INFO(this->get_logger(), "RECV-GyroQuaternions: (%f, %f, %f, %f)", msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    //   RCLCPP_INFO(this->get_logger(), "RECV-TIME:'%f'", msg->time_stamp);
    // }
    void pose_stamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
      RCLCPP_INFO(this->get_logger(), "RECV package type [Gimbel Angel PoseStamped]");
      RCLCPP_INFO(this->get_logger(), "RECV-TIME:'%f'", msg->header.stamp.sec+10e-9*msg->header.stamp.nanosec);
      RCLCPP_INFO(this->get_logger(), "RECV-GyroQuaternions: (%f, %f, %f, %f)", msg->pose.orientation.x, 
                    msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    }
    // rclcpp::Subscription<rm_interfaces::msg::GyroQuaternions>::SharedPtr gyro_quaternions_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
/*

rm_interfaces::msg::GyroQuaternions Gyro_msg;
float yaw = 0.0, pitch = 0.0, roll = 0.0;
rm_interfaces::msg::Gyroquaternions Gyro_msg;
packet.unload_data(yaw, 6);
packet.unload_data(pitch, 10);
packet.unload_data(roll, 14);
Gyro_msg.yaw = yaw;
Gyro_msg.pitch = pitch;
Gyro_msg.roll = roll;
RCLCPP_INFO(node_->get_logger(), "RECV package type [Gimbel Angel Position]");
RCLCPP_INFO(node_->get_logger(), "RECV-TID: '%d'", Gyro_msg.tid);
RCLCPP_INFO(node_->get_logger(), "RECV-GyroQuaternions: (%f, %f, %f, %f)", Gyro_msg.q[0], Gyro_msg.q[1], Gyro_msg.q[2], Gyro_msg.q[3]);
RCLCPP_INFO(node_->get_logger(), "RECV-TIME:'%f'", Gyro_msg.time_stamp);

Gyro_msg.tid = recv_tid;
Gyro_msg.q = Q;
Gyro_msg.time_stamp = time_stamp;
gyro_quaternions_pub_->publish(Gyro_msg);
*/