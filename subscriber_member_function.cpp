#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "Eigen/Dense" // Include Eigen for handling matrices

using std::placeholders::_1;

class TransformSubscriber : public rclcpp::Node
{
public:
  TransformSubscriber()
  : Node("transform_subscriber")
  {
    transform_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "transform_topic", 10, std::bind(&TransformSubscriber::transform_callback, this, _1));
  }

private:
  void transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const
  { 
    RCLCPP_INFO(this->get_logger(), "Received Transform - Frame ID: %s, Child Frame ID: %s",
                msg->header.frame_id.c_str(), msg->child_frame_id.c_str()); 
    RCLCPP_INFO(this->get_logger(), "Translation - X: %f, Y: %f, Z: %f",
                msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "Rotation - X: %f, Y: %f, Z: %f, W: %f",
                msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
  }

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransformSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
