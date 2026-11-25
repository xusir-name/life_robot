#include "graspnet_pick/estimator_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "graspnet_pick/geometry.hpp"
#include "graspnet_pick/transforms.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

namespace graspnet_pick {
namespace {
constexpr double kRad2Deg = 57.29577951308232;
}

GraspEstimator::GraspEstimator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("grasp_estimator", options) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", false);
  }

  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
  depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  box_topic_ = declare_parameter<std::string>("box_topic", "/grasp/target_box");
  candidate_topic_ =
      declare_parameter<std::string>("candidate_topic", "/grasp/best_grasp");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/grasp/best_pose");
  camera_frame_ =
      declare_parameter<std::string>("camera_frame", "camera_optical_frame");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  depth_scale_m_ = declare_parameter<double>("depth_scale_m", 0.001);
  stride_ = declare_parameter<int>("stride", 2);
  min_depth_m_ = declare_parameter<double>("min_depth_m", 0.05);
  max_depth_m_ = declare_parameter<double>("max_depth_m", 3.0);
  max_pair_dt_s_ = declare_parameter<double>("max_pair_dt_s", 0.15);
  sensor_timeout_s_ = declare_parameter<double>("sensor_timeout_s", 1.0);
  box_timeout_s_ = declare_parameter<double>("box_timeout_s", 10.0);
  estimate_rate_hz_ = declare_parameter<double>("estimate_rate_hz", 5.0);
  backend_ = declare_parameter<std::string>("backend", "geometric");
  remove_plane_ = declare_parameter<bool>("remove_plane", true);
  plane_distance_m_ = declare_parameter<double>("plane_distance_m", 0.012);
  plane_min_ratio_ = declare_parameter<double>("plane_min_ratio", 0.25);
  plane_min_points_ = declare_parameter<int>("plane_min_points", 80);
  plane_iterations_ = declare_parameter<int>("plane_iterations", 200);
  plane_max_normal_angle_rad_ =
      declare_parameter<double>("plane_max_normal_angle_rad", 0.436);
  voxel_m_ = declare_parameter<double>("voxel_m", 0.003);
  cluster_eps_m_ = declare_parameter<double>("cluster_eps_m", 0.008);
  cluster_min_points_ = declare_parameter<int>("cluster_min_points", 20);
  min_width_m_ = declare_parameter<double>("min_width_m", 0.005);
  max_width_m_ = declare_parameter<double>("max_width_m", 0.09);
  preferred_width_m_ = declare_parameter<double>("preferred_width_m", 0.04);
  nms_translation_m_ = declare_parameter<double>("nms_translation_m", 0.02);
  nms_rotation_rad_ = declare_parameter<double>("nms_rotation_rad", 0.5236);
  max_grasps_ = declare_parameter<int>("max_grasps", 200);
  min_score_ = declare_parameter<double>("min_score", 0.25);

  validateParams();
  if (backend_ != "geometric") {
    throw std::invalid_argument(
        "unsupported backend '" + backend_ +
        "': only 'geometric' is available until the official GraspNet "
        "checkpoint/runtime is installed");
  }

  candidate_pub_ = create_publisher<interfaces::msg::GraspCandidate>(
      candidate_topic_, rclcpp::QoS(10));
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_,
                                                                rclcpp::QoS(10));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GraspEstimator::onImage, this, std::placeholders::_1));
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GraspEstimator::onDepth, this, std::placeholders::_1));
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GraspEstimator::onCameraInfo, this, std::placeholders::_1));
  box_sub_ = create_subscription<sensor_msgs::msg::RegionOfInterest>(
      box_topic_, rclcpp::QoS(10),
      std::bind(&GraspEstimator::onBox, this, std::placeholders::_1));

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(estimate_rate_hz_, 0.1)));
  timer_ = create_wall_timer(period, std::bind(&GraspEstimator::tick, this));

  RCLCPP_INFO(get_logger(),
              "grasp_estimator ready: backend=%s, depth=%s, box=%s, candidate=%s",
              backend_.c_str(), depth_topic_.c_str(), box_topic_.c_str(),
              candidate_topic_.c_str());
}

void GraspEstimator::validateParams() const {
  if (stride_ < 1) {
    throw std::invalid_argument("stride must be >= 1");
  }
  if (max_depth_m_ <= min_depth_m_) {
    throw std::invalid_argument("max_depth_m must be greater than min_depth_m");
  }
  if (estimate_rate_hz_ <= 0.0) {
    throw std::invalid_argument("estimate_rate_hz must be > 0");
  }
  if (!(0.0 < min_width_m_ && min_width_m_ < max_width_m_)) {
    throw std::invalid_argument("require 0 < min_width_m < max_width_m");
  }
  if (preferred_width_m_ < min_width_m_ || preferred_width_m_ > max_width_m_) {
    throw std::invalid_argument(
        "preferred_width_m must be within [min_width_m, max_width_m]");
  }
  if (cluster_min_points_ < 3) {
    throw std::invalid_argument("cluster_min_points must be >= 3");
  }
}

void GraspEstimator::onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  rgb_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
}

void GraspEstimator::onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(*msg, msg->encoding);
  } catch (const cv_bridge::Exception &exc) {
    RCLCPP_ERROR(get_logger(), "depth conversion failed: %s", exc.what());
    return;
  }
  cv::Mat depth = cv_ptr->image;
  if (depth.empty() || depth.dims != 2) {
    RCLCPP_ERROR(get_logger(), "unsupported depth shape");
    return;
  }
  if (depth.type() == CV_16U) {
    cv::Mat converted;
    depth.convertTo(converted, CV_32F, depth_scale_m_);
    depth = converted;
  } else if (depth.type() == CV_64F) {
    cv::Mat converted;
    depth.convertTo(converted, CV_32F);
    depth = converted;
  } else if (depth.type() != CV_32F) {
    RCLCPP_ERROR(get_logger(), "unsupported depth type: %d", depth.type());
    return;
  }
  depth_ = depth.clone();
  depth_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
}

void GraspEstimator::onCameraInfo(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
    RCLCPP_ERROR(get_logger(), "camera_info has invalid fx/fy");
    return;
  }
  K_ << msg->k[0], msg->k[1], msg->k[2],
        msg->k[3], msg->k[4], msg->k[5],
        msg->k[6], msg->k[7], msg->k[8];
  has_K_ = true;
}

void GraspEstimator::onBox(
    const sensor_msgs::msg::RegionOfInterest::SharedPtr msg) {
  if (msg->width == 0 || msg->height == 0) {
    RCLCPP_WARN(get_logger(), "ignore empty ROI");
    return;
  }
  box_ = *msg;
  box_stamp_ = now();
}

void GraspEstimator::tick() {
  if (depth_.empty() || !has_K_ || !box_.has_value() || !depth_stamp_.has_value() ||
      !box_stamp_.has_value()) {
    return;
  }
  const rclcpp::Time now_time = now();
  if ((now_time - *box_stamp_).seconds() > box_timeout_s_) {
    return;
  }
  if ((now_time - *depth_stamp_).seconds() > sensor_timeout_s_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "depth image is stale");
    return;
  }
  if (rgb_stamp_.has_value()) {
    const double pair_dt = std::abs((*rgb_stamp_ - *depth_stamp_).seconds());
    if (pair_dt > max_pair_dt_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "RGB/depth stamp mismatch %.3fs", pair_dt);
      return;
    }
  }

  const int x1 = std::max(0, static_cast<int>(box_->x_offset));
  const int y1 = std::max(0, static_cast<int>(box_->y_offset));
  const int x2 = std::min(depth_.cols,
                          static_cast<int>(box_->x_offset + box_->width));
  const int y2 = std::min(depth_.rows,
                          static_cast<int>(box_->y_offset + box_->height));
  if (x2 <= x1 || y2 <= y1) {
    return;
  }
  const cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
  const auto key = std::make_pair(depth_stamp_->nanoseconds(), roi);
  if (last_estimate_key_.has_value() && *last_estimate_key_ == key) {
    return;
  }

  try {
    estimate(roi);
    last_estimate_key_ = key;
  } catch (const std::exception &exc) {
    RCLCPP_ERROR(get_logger(), "grasp estimation failed: %s", exc.what());
  }
}

std::optional<Eigen::Vector3d> GraspEstimator::tableNormalCamera(
    const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped tf;
  bool found = false;
  try {
    tf = tf_buffer_->lookupTransform(base_frame_, camera_frame_, stamp,
                                     rclcpp::Duration::from_seconds(0.0));
    found = true;
  } catch (const std::exception &) {
    try {
      const rclcpp::Time latest(0, 0, get_clock()->get_clock_type());
      tf = tf_buffer_->lookupTransform(base_frame_, camera_frame_, latest,
                                       rclcpp::Duration::from_seconds(0.0));
      found = true;
    } catch (const std::exception &) {
      found = false;
    }
  }
  if (!found) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "TF unavailable, fallback to PCA OBB");
    return std::nullopt;
  }

  Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                       tf.transform.rotation.y, tf.transform.rotation.z);
  q.normalize();
  const Eigen::Matrix3d R = q.toRotationMatrix();  // camera -> base
  Eigen::Vector3d up = R.row(2).transpose();       // base +Z in camera
  if (up.norm() < 1e-9) {
    return std::nullopt;
  }
  return up.normalized();
}

void GraspEstimator::estimate(const cv::Rect &roi) {
  auto cloud = depthToPoints(depth_, K_, roi, stride_, min_depth_m_, max_depth_m_);
  if (static_cast<int>(cloud.points.size()) < cluster_min_points_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "ROI [%d,%d,%d,%d] has too few depth points (%zu)",
                         roi.x, roi.y, roi.width, roi.height, cloud.points.size());
    return;
  }

  std::optional<Eigen::Vector3d> up = tableNormalCamera(*depth_stamp_);
  std::vector<Eigen::Vector3d> points = std::move(cloud.points);
  if (remove_plane_) {
    auto plane_result = removeDominantPlane(
        points, plane_distance_m_, plane_min_ratio_, plane_min_points_,
        plane_iterations_, 7, up, plane_max_normal_angle_rad_);
    points = std::move(plane_result.points);
  }
  points = voxelDownsample(points, voxel_m_);
  if (static_cast<int>(points.size()) < cluster_min_points_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "ROI [%d,%d,%d,%d] has too few object points after "
                         "filtering (%zu)",
                         roi.x, roi.y, roi.width, roi.height, points.size());
    return;
  }

  GraspSamplingOptions options;
  options.min_width_m = min_width_m_;
  options.max_width_m = max_width_m_;
  options.preferred_width_m = preferred_width_m_;
  options.cluster_eps_m = cluster_eps_m_;
  options.cluster_min_points = cluster_min_points_;
  options.nms_translation_m = nms_translation_m_;
  options.nms_rotation_rad = nms_rotation_rad_;
  options.max_grasps = max_grasps_;
  options.up = up;
  const auto grasps = sampleGrasps(points, options);
  const auto best = selectBestGrasp(grasps, min_score_);
  if (!best.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "no valid grasp in ROI [%d,%d,%d,%d] "
                         "(points=%zu, candidates=%zu)",
                         roi.x, roi.y, roi.width, roi.height, points.size(),
                         grasps.size());
    return;
  }

  const Eigen::Quaterniond q = matrixToQuat(best->rotation);
  interfaces::msg::GraspCandidate candidate;
  candidate.header.stamp = *depth_stamp_;
  candidate.header.frame_id = camera_frame_;
  candidate.pose.position.x = best->position.x();
  candidate.pose.position.y = best->position.y();
  candidate.pose.position.z = best->position.z();
  candidate.pose.orientation.x = q.x();
  candidate.pose.orientation.y = q.y();
  candidate.pose.orientation.z = q.z();
  candidate.pose.orientation.w = q.w();
  candidate.score = static_cast<float>(best->score);
  candidate.width_m = static_cast<float>(best->width_m);
  candidate.source_box = *box_;
  candidate.backend = backend_;
  candidate_pub_->publish(candidate);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = candidate.header;
  pose_msg.pose = candidate.pose;
  pose_pub_->publish(pose_msg);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "best grasp score=%.3f width=%.1fmm p=[%.3f, %.3f, %.3f] "
                       "candidates=%zu points=%zu",
                       best->score, best->width_m * 1000.0, best->position.x(),
                       best->position.y(), best->position.z(), grasps.size(),
                       points.size());
}

}  // namespace graspnet_pick
