#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <optional>
#include <tuple>
#include <vector>

namespace graspnet_pick {

struct PointCloud {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector2i> uv;
};

struct Grasp {
  double score{0.0};
  double width_m{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  int cluster_id{-1};
};

struct Plane {
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double offset{0.0};
};

struct PlaneResult {
  std::vector<Eigen::Vector3d> points;
  std::optional<Plane> plane;
};

struct GraspSamplingOptions {
  double min_width_m{0.005};
  double max_width_m{0.09};
  double preferred_width_m{0.04};
  double cluster_eps_m{0.015};
  int cluster_min_points{20};
  double nms_translation_m{0.02};
  double nms_rotation_rad{0.5235987755982988};
  int max_grasps{200};
  std::optional<Eigen::Vector3d> up;
  // 若给出，只处理质心离该点最近的聚类（一个框一个目标）。
  std::optional<Eigen::Vector3d> preferred_point;
};

PointCloud depthToPoints(const cv::Mat &depth, const Eigen::Matrix3d &K,
                         const std::optional<cv::Rect> &roi, int stride,
                         double min_depth_m, double max_depth_m);

std::vector<Eigen::Vector3d> voxelDownsample(
    const std::vector<Eigen::Vector3d> &points, double voxel_m);

PlaneResult removeDominantPlane(
    const std::vector<Eigen::Vector3d> &points, double distance_threshold_m,
    double min_inlier_ratio, int min_plane_points, int iterations, int seed,
    const std::optional<Eigen::Vector3d> &expected_normal,
    double max_normal_angle_rad);

std::vector<int> clusterPoints(const std::vector<Eigen::Vector3d> &points,
                               double eps_m, int min_points);

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> pcaObb(
    const std::vector<Eigen::Vector3d> &points);

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> tabletopObb(
    const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &up);

std::vector<Grasp> sampleGrasps(const std::vector<Eigen::Vector3d> &points,
                                const GraspSamplingOptions &options);

std::vector<Grasp> nmsGrasps(std::vector<Grasp> grasps,
                             double translation_m, double rotation_rad,
                             int max_grasps);

std::optional<Grasp> selectBestGrasp(const std::vector<Grasp> &grasps,
                                     double min_score);

double rotationAngleRad(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2);

}  // namespace graspnet_pick
