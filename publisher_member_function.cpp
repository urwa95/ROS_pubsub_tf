#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "Eigen/Dense" // Include Eigen for homogeneous matrix

using namespace std::chrono_literals;

class TransformPublisher : public rclcpp::Node
{
public:
  TransformPublisher()
  : Node("transform_publisher"), count_(0)
  {
    transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TransformPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto transform_msg = geometry_msgs::msg::TransformStamped();
    transform_msg.header.stamp = this->now();
    transform_msg.header.frame_id = "drone";
    transform_msg.child_frame_id = "camera";
    
    // Homogeneous transformation matrix
    Eigen::Isometry3d homogeneous_matrix = Eigen::Isometry3d::Identity();
    homogeneous_matrix.translation() << 1.0, 0.0, 0.0; // Translate 1 meter in the x-axis

    // Camera tilting around the y-axis
    double tilt_angle = count_ * 0.1; // Tilt angle increases over time
    homogeneous_matrix.rotate(Eigen::AngleAxisd(tilt_angle, Eigen::Vector3d::UnitY()));

    // Extract translation and rotation from the matrix
    Eigen::Vector3d translation = homogeneous_matrix.translation();
    Eigen::Quaterniond rotation(homogeneous_matrix.rotation());

    // Set transform message values
    transform_msg.transform.translation.x = translation.x();
    transform_msg.transform.translation.y = translation.y();
    transform_msg.transform.translation.z = translation.z();
    transform_msg.transform.rotation.x = rotation.x();
    transform_msg.transform.rotation.y = rotation.y();
    transform_msg.transform.rotation.z = rotation.z();
    transform_msg.transform.rotation.w = rotation.w();

    transform_publisher_->publish(transform_msg);

    RCLCPP_INFO(this->get_logger(), "Published Transform");
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_publisher_;
  size_t count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransformPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
