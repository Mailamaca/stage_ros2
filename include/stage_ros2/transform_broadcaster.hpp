#pragma once

#include <memory>
#include <vector>

#include "tf2_ros/visibility_control.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/qos.hpp"

// The re-implementation here of the standard ROS transform broadcaster is an
// hack to be able to customize the "/tf" topic to a private namespace.
class TransformBroadcaster {
public:
  explicit TransformBroadcaster(const rclcpp::Node::SharedPtr &node);

  void sendTransform(const geometry_msgs::msg::TransformStamped &transform);

  void sendTransform(
      const std::vector<geometry_msgs::msg::TransformStamped> &transforms);

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};
