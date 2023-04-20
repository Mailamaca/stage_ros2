#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stage/stage.hh"
#include "stage_ros2/camera_wrapper.hpp"
#include "stage_ros2/position_wrapper.hpp"
#include "stage_ros2/ranger_wrapper.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

class RobotWrapper {
public:
  RobotWrapper(
      const rclcpp::executors::SingleThreadedExecutor::SharedPtr &executor,
      std::string name);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish(const rclcpp::Time &now);

  void wrap(Stg::Model *mod);

private:
  std::mutex mtx_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string tf_prefix_;
  std::shared_ptr<PositionWrapper> position_;
  std::vector<std::shared_ptr<CameraWrapper>> cameras_;
  std::vector<std::shared_ptr<RangerWrapper>> rangers_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};
