#include "stage_ros2/ranger_wrapper.hpp"

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "stage/stage.hh"
#include "stage_ros2/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

RangerWrapper::RangerWrapper(const rclcpp::Node::SharedPtr &node,
                             Stg::ModelRanger *model,
                             std::string parent_frame_id, std::string frame_id,
                             const std::string &topic)
    : model_(model), parent_frame_id_(std::move(parent_frame_id)),
      frame_id_(std::move(frame_id)),
      is_range_(model_->GetSensors()[0].sample_count == 1) {

  if (is_range_) {
    range_pub_ = node->create_publisher<sensor_msgs::msg::Range>(
        std::string("~/") + topic, 10);
  } else {
    laser_scan_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
        std::string("~/") + topic, 10);
  }
}

void RangerWrapper::publish(
    const std::shared_ptr<TransformBroadcaster> &tf_broadcaster,
    const rclcpp::Time &now) {
  auto sensor = model_->GetSensors()[0];
  if (is_range_) {
    sensor_msgs::msg::Range msg;
    msg.max_range = static_cast<float>(sensor.range.max);
    msg.min_range = static_cast<float>(sensor.range.min);
    msg.field_of_view = static_cast<float>(sensor.fov);
    msg.range = static_cast<float>(sensor.ranges[0]);

    msg.header.frame_id = frame_id_;
    msg.header.stamp = now;
    range_pub_->publish(msg);
  } else {
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
  }
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
