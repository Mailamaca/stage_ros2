#pragma once
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stage/stage.hh"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class PositionWrapper {
public:
  PositionWrapper(const rclcpp::Node::SharedPtr &node,
                  Stg::ModelPosition *model, const std::string &tf_prefix);
  void set_speed(double vx, double vy, double avz);
  void
  publish(const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
          const rclcpp::Time &now);

private:
  Stg::ModelPosition *model_;
  std::string tf_prefix_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
};
