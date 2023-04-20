#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stage/stage.hh"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class CameraWrapper {
public:
  CameraWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelCamera *model,
                const std::string &name, const std::string &tf_prefix);

  void
  publish(const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
          const rclcpp::Time &now);

  void publish_image(const rclcpp::Time &now);
  void publish_depth(const rclcpp::Time &now);
  void publich_camera_info(const rclcpp::Time &now);
  void publish_tf(
      const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
      const rclcpp::Time &now);

private:
  Stg::ModelCamera *model_;
  std::string parent_frame_id_;
  std::string frame_id_;
  bool is_depth_canonical_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};
