#include "stage_ros2/position_wrapper.hpp"

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stage/stage.hh"
#include "stage_ros2/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PositionWrapper::PositionWrapper(const rclcpp::Node::SharedPtr &node,
                                 Stg::ModelPosition *model)
    : model_(model) {
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
  ground_truth_pub_ =
      node->create_publisher<nav_msgs::msg::Odometry>("~/ground_truth", 10);
}

void PositionWrapper::set_speed(double vx, double vy, double avz) {
  model_->SetSpeed(vx, vy, avz);
}

void PositionWrapper::publish(
    const std::shared_ptr<TransformBroadcaster> &tf_broadcaster,
    const rclcpp::Time &now) {
  std::string frame_id = "odom";
  std::string child_frame_id = "base_footprint";

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.pose.pose.position.x = model_->est_pose.x;
  odom_msg.pose.pose.position.y = model_->est_pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, model_->est_pose.a);
  odom_msg.pose.pose.orientation = toMsg(q);
  Stg::Velocity v = model_->GetVelocity();
  odom_msg.twist.twist.linear.x = v.x;
  odom_msg.twist.twist.linear.y = v.y;
  odom_msg.twist.twist.angular.z = v.a;
  odom_msg.header.frame_id = frame_id;
  odom_msg.child_frame_id = child_frame_id;
  odom_msg.header.stamp = now;
  odom_pub_->publish(odom_msg);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom_msg.header;
  transform.child_frame_id = odom_msg.child_frame_id;
  transform.transform.translation.x = odom_msg.pose.pose.position.x;
  transform.transform.translation.y = odom_msg.pose.pose.position.y;
  transform.transform.translation.z = odom_msg.pose.pose.position.z;
  transform.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  transform.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  transform.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  transform.transform.rotation.w = odom_msg.pose.pose.orientation.w;
  tf_broadcaster->sendTransform(transform);

  auto global_pose = model_->GetGlobalPose();
  nav_msgs::msg::Odometry ground_truth_msg;
  ground_truth_msg.pose.pose.position.x = global_pose.x;
  ground_truth_msg.pose.pose.position.y = global_pose.y;
  tf2::Quaternion q_global_pose;
  q_global_pose.setRPY(0.0, 0.0, global_pose.a);
  ground_truth_msg.pose.pose.orientation = toMsg(q_global_pose);
  ground_truth_msg.header.frame_id = frame_id;
  ground_truth_msg.child_frame_id = child_frame_id;
  ground_truth_msg.header.stamp = now;
  ground_truth_pub_->publish(ground_truth_msg);
}
