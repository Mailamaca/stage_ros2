#pragma once
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "stage/stage.hh"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class RangerWrapper {
public:
  RangerWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelRanger *model,
                const std::string &name, const std::string &tf_prefix);

  void
  publish(const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
          const rclcpp::Time &now);

private:
  Stg::ModelRanger *model_;
  std::string parent_frame_id_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  bool is_sonar;
};
