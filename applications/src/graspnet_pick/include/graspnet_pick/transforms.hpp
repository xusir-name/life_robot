#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <utility>

namespace graspnet_pick {

Eigen::Matrix4d transformFromQuat(const Eigen::Vector3d &translation,
                                  const Eigen::Quaterniond &rotation);
Eigen::Matrix4d invertTransform(const Eigen::Matrix4d &T);
Eigen::Matrix4d translationTransform(double x, double y, double z);
Eigen::Quaterniond matrixToQuat(const Eigen::Matrix3d &R);
std::pair<Eigen::Vector3d, Eigen::Quaterniond> matrixToPose(const Eigen::Matrix4d &T);
Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::Pose &pose);
void validateTransform(const Eigen::Matrix4d &T, const std::string &name);

struct Extrinsics {
  Eigen::Matrix4d T_tcp_cam = Eigen::Matrix4d::Identity();
  std::string parent_frame{"tcp"};
  std::string child_frame{"camera_optical_frame"};
};

Extrinsics loadExtrinsicsYaml(const std::string &path);

}  // namespace graspnet_pick
