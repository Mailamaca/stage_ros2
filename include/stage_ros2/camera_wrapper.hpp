#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stage/stage.hh"
#include "stage_ros2/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class CameraWrapper {
public:
  CameraWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelCamera *model,
                std::string parent_frame_id, std::string frame_id,
                const std::string &topic);

  void publish(const std::shared_ptr<TransformBroadcaster> &tf_broadcaster,
               const rclcpp::Time &now);

  void publish_image(const rclcpp::Time &now);
  void publish_depth(const rclcpp::Time &now);
  void publich_camera_info(const rclcpp::Time &now);
  void publish_tf(const std::shared_ptr<TransformBroadcaster> &tf_broadcaster,
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
