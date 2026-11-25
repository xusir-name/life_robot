#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <set>
#include <cstdio>
#include <vector>

#include "graspnet_pick/geometry.hpp"
#include "graspnet_pick/transforms.hpp"

namespace {

using graspnet_pick::Grasp;
using graspnet_pick::GraspSamplingOptions;
using graspnet_pick::clusterPoints;
using graspnet_pick::depthToPoints;
using graspnet_pick::nmsGrasps;
using graspnet_pick::pcaObb;
using graspnet_pick::removeDominantPlane;
using graspnet_pick::rotationAngleRad;
using graspnet_pick::sampleGrasps;
using graspnet_pick::selectBestGrasp;
using graspnet_pick::tabletopObb;
using graspnet_pick::voxelDownsample;

std::vector<Eigen::Vector3d> boxPoints(const Eigen::Vector3d &center,
                                       const Eigen::Vector3d &size,
                                       const Eigen::Matrix3d &rotation,
                                       int n, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-0.5, 0.5);
  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d local(dist(rng) * size.x(), dist(rng) * size.y(),
                                dist(rng) * size.z());
    points.push_back(center + rotation * local);
  }
  return points;
}

}  // namespace

TEST(Geometry, DepthToPointsCenterPixel) {
  const Eigen::Matrix3d K =
      (Eigen::Matrix3d() << 100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0)
          .finished();
  cv::Mat depth(80, 100, CV_32F, cv::Scalar(0.5));
  const auto cloud = depthToPoints(depth, K, cv::Rect(45, 35, 11, 11), 1, 0.05, 3.0);
  ASSERT_EQ(cloud.points.size(), 121U);
  EXPECT_EQ(cloud.uv.size(), cloud.points.size());
  for (const auto &p : cloud.points) {
    EXPECT_NEAR(p.z(), 0.5, 1e-9);
  }
  for (const auto &uv : cloud.uv) {
    EXPECT_GE(uv.x(), 45);
    EXPECT_LT(uv.x(), 56);
    EXPECT_GE(uv.y(), 35);
    EXPECT_LT(uv.y(), 46);
  }
}

TEST(Geometry, DepthToPointsRejectsInvalid) {
  const Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  cv::Mat depth(10, 10, CV_32F, cv::Scalar(0.0));
  const auto cloud = depthToPoints(depth, K, std::nullopt, 1, 0.1, 3.0);
  EXPECT_TRUE(cloud.points.empty());
  EXPECT_TRUE(cloud.uv.empty());
}

TEST(Geometry, RemoveDominantPlane) {
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> dist(-0.3, 0.3);
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 1500; ++i) {
    points.emplace_back(dist(rng), dist(rng), 0.0);
  }
  const auto object = boxPoints(Eigen::Vector3d(0.0, 0.0, 0.05),
                                Eigen::Vector3d(0.05, 0.05, 0.05),
                                Eigen::Matrix3d::Identity(), 300, 3);
  points.insert(points.end(), object.begin(), object.end());

  const auto result = removeDominantPlane(points, 0.003, 0.25, 80, 200, 7,
                                          std::nullopt, 0.436);
  ASSERT_TRUE(result.plane.has_value());
  EXPECT_EQ(result.points.size(), object.size());
}

TEST(Geometry, RemovePlaneRespectsExpectedNormal) {
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> dist(-0.2, 0.2);
  std::vector<Eigen::Vector3d> plane;
  for (int i = 0; i < 800; ++i) {
    plane.emplace_back(0.0, dist(rng), dist(rng));
  }
  const auto result = removeDominantPlane(
      plane, 0.006, 0.25, 80, 200, 7, Eigen::Vector3d(0.0, 0.0, 1.0), 0.349);
  EXPECT_FALSE(result.plane.has_value());
  EXPECT_EQ(result.points.size(), plane.size());
}

TEST(Geometry, PcaObbRecoversBox) {
  const double yaw = 30.0 * M_PI / 180.0;
  Eigen::Matrix3d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), 0.0,
              std::sin(yaw), std::cos(yaw), 0.0,
              0.0, 0.0, 1.0;
  const Eigen::Vector3d center(0.1, -0.2, 0.4);
  const Eigen::Vector3d size(0.08, 0.04, 0.03);
  const auto points = boxPoints(center, size, rotation, 3000, 0);
  const auto [got_center, axes, extents] = pcaObb(points);
  EXPECT_TRUE(got_center.isApprox(center, 3e-3));
  std::vector<double> got{extents.x(), extents.y(), extents.z()};
  std::vector<double> expected{size.x(), size.y(), size.z()};
  std::sort(got.begin(), got.end());
  std::sort(expected.begin(), expected.end());
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(got[i], expected[i], 1e-2);
  }
  EXPECT_TRUE((axes.transpose() * axes).isApprox(Eigen::Matrix3d::Identity(), 1e-8));
  EXPECT_GT(axes.determinant(), 0.0);
}

TEST(Geometry, ClusterTwoObjects) {
  const auto a = boxPoints(Eigen::Vector3d(-0.1, 0.0, 0.5),
                           Eigen::Vector3d(0.02, 0.02, 0.02),
                           Eigen::Matrix3d::Identity(), 100, 4);
  const auto b = boxPoints(Eigen::Vector3d(0.1, 0.0, 0.5),
                           Eigen::Vector3d(0.02, 0.02, 0.02),
                           Eigen::Matrix3d::Identity(), 100, 5);
  std::vector<Eigen::Vector3d> points = a;
  points.insert(points.end(), b.begin(), b.end());
  const auto labels = clusterPoints(points, 0.01, 10);
  std::set<int> unique;
  for (int label : labels) {
    if (label >= 0) {
      unique.insert(label);
    }
  }
  EXPECT_EQ(unique.size(), 2U);
}

TEST(Geometry, TabletopObbSquareSide) {
  const auto points = boxPoints(Eigen::Vector3d(0.0, 0.0, 0.5),
                                Eigen::Vector3d(0.04, 0.04, 0.04),
                                Eigen::Matrix3d::Identity(), 3000, 6);
  const auto [center, axes, extents] =
      tabletopObb(points, Eigen::Vector3d(0.0, 0.0, -1.0));
  EXPECT_TRUE(center.isApprox(Eigen::Vector3d(0.0, 0.0, 0.5), 3e-3));
  std::vector<double> got{extents.x(), extents.y(), extents.z()};
  std::sort(got.begin(), got.end());
  for (double value : got) {
    EXPECT_NEAR(value, 0.04, 4e-3);
  }
  EXPECT_TRUE((axes.transpose() * axes).isApprox(Eigen::Matrix3d::Identity(), 1e-8));
}

TEST(Geometry, SampleGraspsOnBox) {
  const Eigen::Vector3d center(0.0, 0.0, 0.5);
  const Eigen::Vector3d size(0.04, 0.04, 0.04);
  const auto points = boxPoints(center, size, Eigen::Matrix3d::Identity(), 4000, 7);

  GraspSamplingOptions options;
  options.min_width_m = 0.005;
  options.max_width_m = 0.09;
  options.preferred_width_m = 0.04;
  options.cluster_eps_m = 0.01;
  options.cluster_min_points = 20;
  options.up = Eigen::Vector3d(0.0, 0.0, -1.0);
  const auto grasps = sampleGrasps(points, options);
  ASSERT_FALSE(grasps.empty());
  const auto best = selectBestGrasp(grasps, 0.0);
  ASSERT_TRUE(best.has_value());
  EXPECT_TRUE(best->position.isApprox(center, 4e-3));
  EXPECT_GE(best->width_m, 0.03);
  EXPECT_LE(best->width_m, 0.05);
  const Eigen::Vector3d to_object = center.normalized();
  EXPECT_GT(best->rotation.col(2).dot(to_object), 0.5);
  EXPECT_TRUE((best->rotation.transpose() * best->rotation)
                  .isApprox(Eigen::Matrix3d::Identity(), 1e-8));
  EXPECT_GT(best->rotation.determinant(), 0.0);
}

TEST(Geometry, NmsAndRotationAngle) {
  Grasp g1;
  g1.score = 0.9;
  g1.width_m = 0.04;
  g1.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  g1.rotation = Eigen::Matrix3d::Identity();
  Grasp g2 = g1;
  g2.score = 0.8;
  g2.position = Eigen::Vector3d(0.001, 0.0, 0.5);
  Grasp g3 = g1;
  g3.score = 0.7;
  g3.position = Eigen::Vector3d(0.05, 0.0, 0.5);

  const auto kept = nmsGrasps({g1, g2, g3}, 0.02, 30.0 * M_PI / 180.0, 10);
  EXPECT_EQ(kept.size(), 2U);
  EXPECT_LT(rotationAngleRad(Eigen::Matrix3d::Identity(),
                            Eigen::Matrix3d::Identity()),
            1e-8);
}

TEST(Transforms, RoundTrip) {
  const Eigen::Quaterniond q(0.9689124217106447, 0.0, 0.0, 0.24740395925452294);
  const Eigen::Matrix4d T =
      graspnet_pick::transformFromQuat(Eigen::Vector3d(0.1, 0.2, 0.3), q);
  graspnet_pick::validateTransform(T, "T");
  const Eigen::Matrix4d T_inv = graspnet_pick::invertTransform(T);
  EXPECT_TRUE((T * T_inv).isApprox(Eigen::Matrix4d::Identity(), 1e-12));
  const Eigen::Quaterniond got = graspnet_pick::matrixToQuat(T.block<3, 3>(0, 0));
  EXPECT_TRUE(got.toRotationMatrix().isApprox(T.block<3, 3>(0, 0), 1e-12));
}

TEST(Transforms, LoadExtrinsicsVariants) {
  const Eigen::Matrix4d T =
      graspnet_pick::transformFromQuat(Eigen::Vector3d(0.01, -0.02, 0.03),
                                       Eigen::Quaterniond::Identity());
  const std::string path = "/tmp/graspnet_pick_test_extrinsics.yaml";
  {
    std::ofstream out(path);
    out << "parent_frame: tcp\nchild_frame: camera_optical_frame\n";
    out << "transform:\n";
    for (int r = 0; r < 4; ++r) {
      out << "  - [";
      for (int c = 0; c < 4; ++c) {
        out << T(r, c) << (c == 3 ? "]\n" : ", ");
      }
    }
  }
  const auto extrinsics = graspnet_pick::loadExtrinsicsYaml(path);
  EXPECT_EQ(extrinsics.parent_frame, "tcp");
  EXPECT_EQ(extrinsics.child_frame, "camera_optical_frame");
  EXPECT_TRUE(extrinsics.T_tcp_cam.isApprox(T, 1e-12));
  std::remove(path.c_str());
}

TEST(Transforms, HandEyeChain) {
  const Eigen::Matrix4d T_base_tcp = graspnet_pick::transformFromQuat(
      Eigen::Vector3d(0.3, 0.0, 0.4), Eigen::Quaterniond::Identity());
  const Eigen::Matrix4d T_tcp_cam = graspnet_pick::transformFromQuat(
      Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Quaterniond::Identity());
  const Eigen::Matrix4d T_cam_grasp =
      graspnet_pick::translationTransform(0.0, 0.0, 0.5);
  const Eigen::Matrix4d T_base_grasp =
      T_base_tcp * T_tcp_cam * T_cam_grasp;
  EXPECT_TRUE((T_base_grasp.block<3, 1>(0, 3)
                   .isApprox(Eigen::Vector3d(0.3, 0.0, 1.0), 1e-12)));
}
