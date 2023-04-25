#pragma once
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stage/stage.hh"
#include "stage_ros2/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PositionWrapper {
public:
  PositionWrapper(const rclcpp::Node::SharedPtr &node,
                  Stg::ModelPosition *model);
  void set_speed(double vx, double vy, double avz);
  void publish(const std::shared_ptr<TransformBroadcaster> &tf_broadcaster,
               const rclcpp::Time &now);

private:
  Stg::ModelPosition *model_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
};
