#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/msg/grasp_candidate.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

namespace graspnet_pick {

class GraspEstimator : public rclcpp::Node {
 public:
  explicit GraspEstimator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void onBox(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg);
  void tick();
  void estimate(const cv::Rect &roi);
  std::optional<Eigen::Vector3d> tableNormalCamera(const rclcpp::Time &stamp);
  void validateParams() const;

  // topics / frames
  std::string image_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string box_topic_;
  std::string candidate_topic_;
  std::string pose_topic_;
  std::string camera_frame_;
  std::string base_frame_;

  // estimation parameters
  double depth_scale_m_{0.001};
  int stride_{2};
  double min_depth_m_{0.05};
  double max_depth_m_{3.0};
  double max_pair_dt_s_{0.15};
  double sensor_timeout_s_{1.0};
  double box_timeout_s_{10.0};
  double estimate_rate_hz_{5.0};
  std::string backend_{"geometric"};
  bool remove_plane_{true};
  double plane_distance_m_{0.012};
  double plane_min_ratio_{0.25};
  int plane_min_points_{80};
  int plane_iterations_{200};
  double plane_max_normal_angle_rad_{0.436};
  double voxel_m_{0.003};
  double cluster_eps_m_{0.008};
  int cluster_min_points_{20};
  double min_width_m_{0.005};
  double max_width_m_{0.09};
  double preferred_width_m_{0.04};
  double grasp_depth_from_top_m_{0.01};
  double nms_translation_m_{0.02};
  double nms_rotation_rad_{0.5236};
  int max_grasps_{200};
  double min_score_{0.25};

  // state
  cv::Mat depth_;
  std::optional<rclcpp::Time> depth_stamp_;
  std::optional<rclcpp::Time> rgb_stamp_;
  Eigen::Matrix3d K_{Eigen::Matrix3d::Identity()};
  bool has_K_{false};
  std::optional<sensor_msgs::msg::RegionOfInterest> box_;
  std::optional<rclcpp::Time> box_stamp_;
  std::optional<std::pair<int64_t, cv::Rect>> last_estimate_key_;

  rclcpp::Publisher<interfaces::msg::GraspCandidate>::SharedPtr candidate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr box_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace graspnet_pick
