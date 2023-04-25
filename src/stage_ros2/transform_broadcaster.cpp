#include "stage_ros2/transform_broadcaster.hpp"

#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

TransformBroadcaster::TransformBroadcaster(
    const rclcpp::Node::SharedPtr &node) {
  publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "~/tf", tf2_ros::DynamicBroadcasterQoS());
}

void TransformBroadcaster::sendTransform(
    const geometry_msgs::msg::TransformStamped &transform) {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(transform);
  sendTransform(transforms);
}

void TransformBroadcaster::sendTransform(
    const std::vector<geometry_msgs::msg::TransformStamped> &transforms) {
  tf2_msgs::msg::TFMessage message;
  for (const auto &transform : transforms) {
    message.transforms.push_back(transform);
  }
  publisher_->publish(message);
}
