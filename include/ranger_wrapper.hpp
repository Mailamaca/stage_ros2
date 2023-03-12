#pragma once
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "stage.hh"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class RangerWrapper
{
public:
  RangerWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelRanger *model,
                const std::string &name, const std::string &tf_prefix)
      : model_(model) {
    parent_frame_id_ = "base_link";
    if (tf_prefix.size() > 0) {
      parent_frame_id_ = tf_prefix + "/" + parent_frame_id_;
    }
    frame_id_ = name + "/base_scan";
    if (tf_prefix.size() > 0) {
      frame_id_ = tf_prefix + "/" + frame_id_;
    }
    laser_scan_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
        std::string("~/") + name, 10);
    is_sonar = model_->GetSensors()[0].sample_count == 1;
    if (is_sonar) {
      RCLCPP_WARN_STREAM(node->get_logger(), "sonar is not supported");
    }
  }

  void
  publish(const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
          const rclcpp::Time &now) {
    if (is_sonar) {
      return;
    }
    auto sensor = model_->GetSensors()[0];
    sensor_msgs::msg::LaserScan msg;
    msg.angle_max = static_cast<float>(sensor.fov / 2.0);
    msg.angle_min = -static_cast<float>(sensor.fov / 2.0);
    msg.angle_increment = static_cast<float>(
        sensor.fov / static_cast<double>(sensor.sample_count - 1));
    msg.range_max = static_cast<float>(sensor.range.max);
    msg.range_min = static_cast<float>(sensor.range.min);
    msg.ranges.resize(sensor.ranges.size());
    msg.intensities.resize(sensor.intensities.size());
    for (unsigned int i = 0; i < sensor.ranges.size(); i++) {
      msg.ranges[i] = static_cast<float>(sensor.ranges[i]);
      msg.intensities[i] = static_cast<float>(sensor.intensities[i]);
    }
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now;
    laser_scan_pub_->publish(msg);

    Stg::Pose p = model_->GetPose();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.a);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent_frame_id_;
    transform.header.stamp = now;
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = p.x;
    transform.transform.translation.y = p.y;
    transform.transform.translation.z = p.z;
    transform.transform.rotation = toMsg(q);
    tf_broadcaster->sendTransform(transform);
  }
  Stg::ModelRanger *model_;
  std::string parent_frame_id_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  bool is_sonar;
};
