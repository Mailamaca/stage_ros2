#pragma once
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "stage/stage.hh"
#include "stage_ros2/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class RangerWrapper {
public:
  RangerWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelRanger *model,
                std::string parent_frame_id, std::string frame_id,
                const std::string &topic);

  void publish(const std::shared_ptr<TransformBroadcaster> &tf_broadcaster,
               const rclcpp::Time &now);

private:
  Stg::ModelRanger *model_;
  std::string parent_frame_id_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_ =
      nullptr;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_ = nullptr;
  bool is_range_;
};
