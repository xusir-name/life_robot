#include "graspnet_pick/executor_node.hpp"

#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "graspnet_pick/transforms.hpp"

namespace graspnet_pick {
namespace {

std::vector<std::string> splitJointNames(const std::string &value) {
  std::vector<std::string> out;
  std::stringstream ss(value);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto first = item.find_first_not_of(" \t");
    const auto last = item.find_last_not_of(" \t");
    if (first != std::string::npos) {
      out.push_back(item.substr(first, last - first + 1));
    }
  }
  return out;
}

bool startsWith(const std::string &value, const char *prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool isTerminalStatus(const std::string &status) {
  return startsWith(status, "succeeded") || startsWith(status, "failed") ||
         startsWith(status, "rejected") || startsWith(status, "busy");
}

}  // namespace

GraspExecutor::GraspExecutor(const rclcpp::NodeOptions &options)
    : rclcpp::Node("grasp_executor", options) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", false);
  }

  extrinsics_yaml_ = declare_parameter<std::string>(
      "extrinsics_yaml",
      "~/life_robot/config/calibration/camera_tcp_extrinsics.yaml");
  candidate_topic_ =
      declare_parameter<std::string>("candidate_topic", "/grasp/best_grasp");
  service_name_ = declare_parameter<std::string>("service_name", "/grasp/execute");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  ee_frame_ = declare_parameter<std::string>("ee_frame", "link6");
  camera_frame_ =
      declare_parameter<std::string>("camera_frame", "camera_optical_frame");
  tcp_topic_ = declare_parameter<std::string>("tcp_topic", "/arm_tcp_state");
  motion_topic_ = declare_parameter<std::string>("motion_topic", "/target_pose");
  motion_status_topic_ =
      declare_parameter<std::string>("motion_status_topic", "/target_pose_status");
  gripper_topic_ = declare_parameter<std::string>(
      "gripper_topic", "/gripper_controller/joint_trajectory");
  gripper_joints_ = splitJointNames(declare_parameter<std::string>(
      "gripper_joints", "gripper_joint1,gripper_joint2"));
  joint_states_topic_ =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");

  max_candidate_age_s_ = declare_parameter<double>("max_candidate_age_s", 3.0);
  max_stamp_diff_s_ = declare_parameter<double>("max_stamp_diff_s", 0.15);
  motion_timeout_s_ = declare_parameter<double>("motion_timeout_s", 30.0);
  tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 10.0);
  settle_time_s_ = declare_parameter<double>("settle_time_s", 0.4);
  finger_length_m_ = declare_parameter<double>("finger_length_m", 0.0);
  declare_parameter<double>("grasp_height_offset_m", 0.0);
  pregrasp_distance_m_ = declare_parameter<double>("pregrasp_distance_m", 0.08);
  retreat_distance_m_ = declare_parameter<double>("retreat_distance_m", 0.08);
  lift_height_m_ = declare_parameter<double>("lift_height_m", 0.05);
  gripper_open_m_ = declare_parameter<double>("gripper_open_m", 0.08);
  gripper_margin_m_ = declare_parameter<double>("gripper_margin_m", 0.004);
  gripper_max_opening_m_ =
      declare_parameter<double>("gripper_max_opening_m", 0.095);
  gripper_joint_limit_m_ =
      declare_parameter<double>("gripper_joint_limit_m", 0.05);
  gripper_move_time_s_ = declare_parameter<double>("gripper_move_time_s", 1.0);
  gripper_settle_s_ = declare_parameter<double>("gripper_settle_s", 1.5);
  min_grasp_opening_m_ =
      declare_parameter<double>("min_grasp_opening_m", 0.005);
  prefer_vertical_approach_ =
      declare_parameter<bool>("prefer_vertical_approach", true);
  validateParams();

  const Extrinsics extrinsics = loadExtrinsicsYaml(extrinsics_yaml_);
  T_tcp_cam_ = extrinsics.T_tcp_cam;
  extrinsics_parent_ = extrinsics.parent_frame;
  extrinsics_child_ = extrinsics.child_frame;
  RCLCPP_INFO(get_logger(),
              "loaded hand-eye %s <- %s from %s: p=[%.4f, %.4f, %.4f]",
              extrinsics_parent_.c_str(), extrinsics_child_.c_str(),
              extrinsics_yaml_.c_str(), T_tcp_cam_(0, 3), T_tcp_cam_(1, 3),
              T_tcp_cam_(2, 3));

  callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  motion_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      motion_topic_, rclcpp::QoS(10));
  gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      gripper_topic_, rclcpp::QoS(10));
  candidate_sub_ = create_subscription<interfaces::msg::GraspCandidate>(
      candidate_topic_, rclcpp::QoS(10),
      std::bind(&GraspExecutor::onCandidate, this, std::placeholders::_1),
      sub_options);
  tcp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      tcp_topic_, rclcpp::QoS(50),
      std::bind(&GraspExecutor::onTcp, this, std::placeholders::_1),
      sub_options);
  motion_status_sub_ = create_subscription<std_msgs::msg::String>(
      motion_status_topic_, rclcpp::QoS(10),
      std::bind(&GraspExecutor::onMotionStatus, this, std::placeholders::_1),
      sub_options);
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(50),
      std::bind(&GraspExecutor::onJointStates, this, std::placeholders::_1),
      sub_options);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  T_ee_tcp_ = lookupEeTcp();
  {
    std::ostringstream ss;
    ss << T_ee_tcp_.block<3, 4>(0, 0);
    RCLCPP_INFO(get_logger(), "T_%s_%s =\n%s", ee_frame_.c_str(),
                extrinsics_parent_.c_str(), ss.str().c_str());
  }

  service_ = create_service<ExecuteGrasp>(
      service_name_,
      std::bind(&GraspExecutor::onExecute, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

  RCLCPP_INFO(get_logger(),
              "grasp_executor ready: service=%s, motion=%s, gripper=%s",
              service_name_.c_str(), motion_topic_.c_str(),
              gripper_topic_.c_str());
}

void GraspExecutor::validateParams() const {
  if (max_candidate_age_s_ <= 0.0) {
    throw std::invalid_argument("max_candidate_age_s must be > 0");
  }
  if (max_stamp_diff_s_ <= 0.0) {
    throw std::invalid_argument("max_stamp_diff_s must be > 0");
  }
  if (motion_timeout_s_ <= 0.0) {
    throw std::invalid_argument("motion_timeout_s must be > 0");
  }
  if (gripper_joints_.size() != 2) {
    throw std::invalid_argument("gripper_joints must contain exactly two joints");
  }
  if (!(0.0 < gripper_max_opening_m_ &&
        gripper_max_opening_m_ <= 2.0 * gripper_joint_limit_m_)) {
    throw std::invalid_argument(
        "require 0 < gripper_max_opening_m <= 2 * gripper_joint_limit_m");
  }
  if (gripper_open_m_ > gripper_max_opening_m_) {
    throw std::invalid_argument(
        "gripper_open_m must not exceed gripper_max_opening_m");
  }
  if (gripper_settle_s_ < gripper_move_time_s_) {
    throw std::invalid_argument(
        "gripper_settle_s must be >= gripper_move_time_s; otherwise joint "
        "feedback is read before the gripper stops");
  }
}

Eigen::Matrix4d GraspExecutor::lookupEeTcp() {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(tf_timeout_s_);
  std::string last_error = "not attempted";
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
          ee_frame_, camera_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.0));
      const Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                 tf.transform.rotation.y, tf.transform.rotation.z);
      const Eigen::Matrix4d T_ee_cam = transformFromQuat(
          Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y,
                          tf.transform.translation.z),
          q);
      const Eigen::Matrix4d T_ee_tcp = T_ee_cam * invertTransform(T_tcp_cam_);
      validateTransform(T_ee_tcp, "T_ee_tcp");
      return T_ee_tcp;
    } catch (const std::exception &exc) {
      last_error = exc.what();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  throw std::runtime_error("TF lookup " + ee_frame_ + " <- " + camera_frame_ +
                           " failed: " + last_error);
}

void GraspExecutor::onCandidate(
    const interfaces::msg::GraspCandidate::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  candidate_ = *msg;
}

void GraspExecutor::onTcp(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  tcp_buffer_.push_back(*msg);
  if (tcp_buffer_.size() > 1000) {
    tcp_buffer_.pop_front();
  }
  state_cv_.notify_all();
}

void GraspExecutor::onMotionStatus(const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  motion_status_ = msg->data;
  has_motion_status_ = true;
  state_cv_.notify_all();
}

void GraspExecutor::onJointStates(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (std::find(gripper_joints_.begin(), gripper_joints_.end(), msg->name[i]) !=
        gripper_joints_.end()) {
      joint_positions_[msg->name[i]] = msg->position[i];
    }
  }
}

void GraspExecutor::onExecute(
    const std::shared_ptr<ExecuteGrasp::Request> request,
    std::shared_ptr<ExecuteGrasp::Response> response) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (busy_) {
      response->success = false;
      response->error_code = ExecuteGrasp::Response::ERROR_BUSY;
      response->message = "another grasp is running";
      return;
    }
    busy_ = true;
  }

  try {
    execute(*request, *response);
  } catch (const std::exception &exc) {
    RCLCPP_ERROR(get_logger(), "grasp execution crashed: %s", exc.what());
    response->success = false;
    response->error_code = ExecuteGrasp::Response::ERROR_INVALID_GRASP;
    response->message = std::string("internal error: ") + exc.what();
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  busy_ = false;
}

void GraspExecutor::fail(ExecuteGrasp::Response &response, uint8_t code,
                         const std::string &message) {
  response.success = false;
  response.error_code = code;
  response.message = message;
  RCLCPP_ERROR(get_logger(), "grasp failed: %s", message.c_str());
}

void GraspExecutor::execute(const ExecuteGrasp::Request &request,
                            ExecuteGrasp::Response &response) {
  std::optional<interfaces::msg::GraspCandidate> candidate;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    candidate = candidate_;
  }
  if (!candidate.has_value()) {
    fail(response, ExecuteGrasp::Response::ERROR_NO_CANDIDATE,
         "no GraspCandidate received yet");
    return;
  }

  const double max_age =
      request.max_age_s > 0.0f ? request.max_age_s : max_candidate_age_s_;
  const double timeout =
      request.timeout_s > 0.0f ? request.timeout_s : motion_timeout_s_;
  const rclcpp::Time stamp(candidate->header.stamp,
                           get_clock()->get_clock_type());
  if (stamp.nanoseconds() == 0) {
    fail(response, ExecuteGrasp::Response::ERROR_INVALID_GRASP,
         "candidate has zero timestamp");
    return;
  }
  const double age = (now() - stamp).seconds();
  if (age > max_age) {
    std::ostringstream ss;
    ss << "candidate age " << age << "s exceeds " << max_age << "s";
    fail(response, ExecuteGrasp::Response::ERROR_STALE_CANDIDATE, ss.str());
    return;
  }
  if (age < -1.0) {
    RCLCPP_WARN(get_logger(),
                "candidate stamp is %.3fs in the future; check use_sim_time / "
                "clock sources",
                -age);
  }

  const auto tcp_match = nearestTcp(stamp);
  if (!tcp_match.has_value()) {
    std::ostringstream ss;
    ss << "no TCP feedback within " << max_stamp_diff_s_
       << "s of candidate stamp";
    fail(response, ExecuteGrasp::Response::ERROR_TRANSFORM, ss.str());
    return;
  }
  const auto &tcp_msg = tcp_match->first;
  const double tcp_dt = tcp_match->second;

  Eigen::Matrix4d T_base_tcp_grasp;
  Eigen::Matrix4d T_base_tcp_image;
  try {
    const Eigen::Matrix4d T_cam_grasp = poseMsgToMatrix(candidate->pose);
    validateTransform(T_cam_grasp, "T_cam_grasp");
    T_base_tcp_image = poseMsgToMatrix(tcp_msg.pose);
    const Eigen::Matrix4d T_base_tcp_fingertip =
        T_base_tcp_image * T_tcp_cam_ * T_cam_grasp;
    T_base_tcp_grasp =
        T_base_tcp_fingertip * translationTransform(0.0, 0.0, -finger_length_m_);
    const double height_offset =
        get_parameter("grasp_height_offset_m").as_double();
    T_base_tcp_grasp(2, 3) += height_offset;
  } catch (const std::exception &exc) {
    fail(response, ExecuteGrasp::Response::ERROR_TRANSFORM,
         std::string("transform failed: ") + exc.what());
    return;
  }

  const double width = candidate->width_m;
  if (!(width > 0.0 && width <= gripper_max_opening_m_)) {
    std::ostringstream ss;
    ss << "grasp width " << width << "m is outside gripper range";
    fail(response, ExecuteGrasp::Response::ERROR_INVALID_GRASP, ss.str());
    return;
  }

  const double open_width =
      std::min(gripper_max_opening_m_,
               std::max(gripper_open_m_, width + gripper_margin_m_));
  RCLCPP_INFO(get_logger(),
              "execute grasp: score=%.3f width=%.1fmm open=%.1fmm "
              "tcp_dt=%.3fs age=%.3fs",
              candidate->score, width * 1000.0, open_width * 1000.0, tcp_dt, age);

  if (!commandGripper(open_width, "open")) {
    fail(response, ExecuteGrasp::Response::ERROR_GRIPPER,
         "failed to command gripper open");
    return;
  }

  const auto rotations = orientationCandidates(
      T_base_tcp_grasp.block<3, 3>(0, 0), T_base_tcp_image.block<3, 3>(0, 0));
  std::optional<Eigen::Matrix4d> chosen_tcp;
  for (std::size_t index = 0; index < rotations.size(); ++index) {
    Eigen::Matrix4d T_base_tcp = T_base_tcp_grasp;
    T_base_tcp.block<3, 3>(0, 0) = rotations[index];
    const Eigen::Vector3d approach_z = rotations[index].col(2);
    RCLCPP_INFO(get_logger(), "orientation[%zu] approach=[%.2f, %.2f, %.2f]",
                index, approach_z.x(), approach_z.y(), approach_z.z());
    const Eigen::Matrix4d T_base_ee = T_base_tcp * invertTransform(T_ee_tcp_);
    if (pregrasp_distance_m_ > 0.0) {
      const Eigen::Matrix4d T_base_tcp_pre =
          T_base_tcp *
          translationTransform(0.0, 0.0, -pregrasp_distance_m_);
      const Eigen::Matrix4d T_base_ee_pre =
          T_base_tcp_pre * invertTransform(T_ee_tcp_);
      std::ostringstream label;
      label << "pregrasp[" << index << "]";
      if (!moveTo(T_base_ee_pre, label.str(), timeout)) {
        continue;
      }
    } else {
      std::ostringstream label;
      label << "grasp[" << index << "]";
      if (!moveTo(T_base_ee, label.str(), timeout)) {
        continue;
      }
    }
    chosen_tcp = T_base_tcp;
    break;
  }

  if (!chosen_tcp.has_value()) {
    commandGripper(open_width, "safe-open");
    fail(response, ExecuteGrasp::Response::ERROR_MOTION,
         "pregrasp motion failed for all orientation candidates");
    return;
  }

  const Eigen::Matrix4d T_base_ee_grasp =
      *chosen_tcp * invertTransform(T_ee_tcp_);
  if (!moveTo(T_base_ee_grasp, "grasp", timeout)) {
    commandGripper(open_width, "safe-open");
    fail(response, ExecuteGrasp::Response::ERROR_MOTION,
         "grasp motion failed or timed out");
    return;
  }
  if (!commandGripper(0.0, "close")) {
    fail(response, ExecuteGrasp::Response::ERROR_GRIPPER,
         "failed to command gripper close");
    return;
  }
  const auto opening = gripperOpening();
  if (opening.has_value()) {
    RCLCPP_INFO(get_logger(), "gripper opening after close: %.1fmm",
                *opening * 1000.0);
  }
  if (opening.has_value() && *opening < min_grasp_opening_m_) {
    std::ostringstream ss;
    ss << "gripper closed empty (opening=" << *opening * 1000.0 << "mm)";
    commandGripper(open_width, "safe-open");
    fail(response, ExecuteGrasp::Response::ERROR_GRIPPER, ss.str());
    return;
  }

  Eigen::Matrix4d T_base_tcp_retreat =
      *chosen_tcp *
      translationTransform(0.0, 0.0, -std::max(0.0, retreat_distance_m_));
  T_base_tcp_retreat(2, 3) += std::max(0.0, lift_height_m_);
  const Eigen::Matrix4d T_base_ee_retreat =
      T_base_tcp_retreat * invertTransform(T_ee_tcp_);
  if (!moveTo(T_base_ee_retreat, "retreat", timeout)) {
    fail(response, ExecuteGrasp::Response::ERROR_MOTION,
         "retreat motion failed; object may still be held");
    return;
  }

  response.success = true;
  response.error_code = ExecuteGrasp::Response::ERROR_NONE;
  std::ostringstream ss;
  ss << "grasp succeeded: score=" << candidate->score << " width=" << width << "m";
  response.message = ss.str();
  response.score = candidate->score;
  RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
}

std::optional<std::pair<geometry_msgs::msg::PoseStamped, double>>
GraspExecutor::nearestTcp(const rclcpp::Time &target) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::optional<geometry_msgs::msg::PoseStamped> best;
  std::optional<double> best_dt;
  for (auto it = tcp_buffer_.rbegin(); it != tcp_buffer_.rend(); ++it) {
    const rclcpp::Time msg_stamp(it->header.stamp, get_clock()->get_clock_type());
    if (msg_stamp.nanoseconds() == 0) {
      continue;
    }
    const double dt = std::abs((msg_stamp - target).seconds());
    if (!best_dt.has_value() || dt < *best_dt) {
      best = *it;
      best_dt = dt;
    }
  }
  if (!best.has_value() || !best_dt.has_value() || *best_dt > max_stamp_diff_s_) {
    return std::nullopt;
  }
  return std::make_pair(*best, *best_dt);
}

std::optional<std::string> GraspExecutor::waitMotionDone(double timeout_s) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (true) {
    if (has_motion_status_ && isTerminalStatus(motion_status_)) {
      return motion_status_;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return std::nullopt;
    }
    state_cv_.wait_for(lock, std::chrono::milliseconds(50));
  }
}

bool GraspExecutor::moveTo(const Eigen::Matrix4d &T_base_ee,
                           const std::string &label, double timeout_s) {
  const auto pose = matrixToPose(T_base_ee);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = base_frame_;
  msg.header.stamp = now();
  msg.pose.position.x = pose.first.x();
  msg.pose.position.y = pose.first.y();
  msg.pose.position.z = pose.first.z();
  msg.pose.orientation.x = pose.second.x();
  msg.pose.orientation.y = pose.second.y();
  msg.pose.orientation.z = pose.second.z();
  msg.pose.orientation.w = pose.second.w();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_motion_status_ = false;
    motion_status_.clear();
  }
  RCLCPP_INFO(get_logger(), "%s: p=[%.3f, %.3f, %.3f]", label.c_str(),
              pose.first.x(), pose.first.y(), pose.first.z());
  motion_pub_->publish(msg);
  const auto status = waitMotionDone(timeout_s);
  if (!status.has_value()) {
    RCLCPP_ERROR(get_logger(), "%s: motion timeout after %.1fs", label.c_str(),
                 timeout_s);
    return false;
  }
  if (!startsWith(*status, "succeeded")) {
    RCLCPP_ERROR(get_logger(), "%s: motion status '%s'", label.c_str(),
                 status->c_str());
    return false;
  }
  std::this_thread::sleep_for(
      std::chrono::duration<double>(std::max(0.0, settle_time_s_)));
  return true;
}

bool GraspExecutor::commandGripper(double opening_m, const std::string &label) {
  const double opening =
      std::clamp(opening_m, 0.0, gripper_max_opening_m_);
  const double half = opening * 0.5;
  if (half > gripper_joint_limit_m_ + 1e-9) {
    RCLCPP_ERROR(get_logger(),
                 "gripper %s: opening %.4fm exceeds joint limit",
                 label.c_str(), opening);
    return false;
  }

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = gripper_joints_;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {half, -half};
  const double move_time = std::max(0.1, gripper_move_time_s_);
  point.time_from_start.sec = static_cast<int32_t>(move_time);
  point.time_from_start.nanosec = static_cast<uint32_t>(
      (move_time - static_cast<double>(point.time_from_start.sec)) * 1e9);
  traj.points.push_back(point);
  gripper_pub_->publish(traj);
  RCLCPP_INFO(get_logger(), "gripper %s: opening=%.1fmm", label.c_str(),
              opening * 1000.0);
  std::this_thread::sleep_for(
      std::chrono::duration<double>(std::max(0.0, gripper_settle_s_)));
  return true;
}

std::optional<double> GraspExecutor::gripperOpening() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<double> values;
  for (const auto &name : gripper_joints_) {
    const auto it = joint_positions_.find(name);
    if (it == joint_positions_.end()) {
      RCLCPP_WARN(get_logger(),
                  "gripper joint feedback unavailable; skip grasp verification");
      return std::nullopt;
    }
    values.push_back(it->second);
  }
  return std::abs(values[0] - values[1]);
}

std::optional<Eigen::Matrix3d> GraspExecutor::rotationWithApproach(
    const Eigen::Vector3d &approach, const Eigen::Vector3d &closing) {
  Eigen::Vector3d z = approach;
  if (z.norm() < 1e-9) {
    return std::nullopt;
  }
  z.normalize();
  Eigen::Vector3d y = closing;
  y -= y.dot(z) * z;
  if (y.norm() < 1e-9) {
    return std::nullopt;
  }
  y.normalize();
  Eigen::Vector3d x = y.cross(z);
  if (x.norm() < 1e-9) {
    return std::nullopt;
  }
  x.normalize();
  y = z.cross(x);
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  if (R.determinant() < 0.0) {
    R.col(0) *= -1.0;
  }
  return R;
}

std::vector<Eigen::Matrix3d> GraspExecutor::orientationCandidates(
    const Eigen::Matrix3d &R_grasp, const Eigen::Matrix3d &R_current) {
  std::vector<Eigen::Matrix3d> candidates;

  // 优先“夹爪朝下”：先纯垂直，再向 +X 倾斜 15/30/45°，避免大夹爪水平
  // 进入时顶到突台/桌面。每个倾角尝试当前偏航和候选偏航。
  if (prefer_vertical_approach_) {
    const std::vector<double> tilts_deg{0.0, 15.0, 30.0, 45.0};
    const std::vector<Eigen::Vector3d> closings{
        R_current.col(1), R_current.col(0), R_grasp.col(1),
        Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)};
    for (std::size_t ti = 0; ti < tilts_deg.size(); ++ti) {
      const double tilt = tilts_deg[ti] * M_PI / 180.0;
      const Eigen::Vector3d approach(std::sin(tilt), 0.0, -std::cos(tilt));
      const std::size_t close_count = ti <= 1 ? 2U : 1U;
      for (std::size_t ci = 0; ci < close_count; ++ci) {
        auto R = rotationWithApproach(approach, closings[ci]);
        if (R.has_value()) {
          candidates.push_back(*R);
        }
      }
    }
  }

  // 估计器给出的朝向（通常沿相机射线，约 45° 下压）。
  candidates.push_back(R_grasp);
  const Eigen::Vector3d approach = R_grasp.col(2);
  for (const auto &closing : {R_current.col(1), R_current.col(0)}) {
    auto R = rotationWithApproach(approach, closing);
    if (R.has_value()) {
      candidates.push_back(*R);
    }
  }
  {
    auto R = rotationWithApproach(R_current.col(2), R_grasp.col(1));
    if (R.has_value()) {
      candidates.push_back(*R);
    }
  }
  candidates.push_back(R_current);

  // 最后才退回水平侧抓。
  for (const auto &horizontal :
       {Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, -1.0, 0.0)}) {
    auto R = rotationWithApproach(horizontal, Eigen::Vector3d(0.0, 1.0, 0.0));
    if (R.has_value()) {
      candidates.push_back(*R);
    }
  }
  Eigen::Matrix3d y_neg;
  y_neg.col(0) = -R_grasp.col(0);
  y_neg.col(1) = -R_grasp.col(1);
  y_neg.col(2) = R_grasp.col(2);
  candidates.push_back(y_neg);

  constexpr double kDedupAngle = 5.0 * 3.14159265358979323846 / 180.0;
  std::vector<Eigen::Matrix3d> unique;
  for (const auto &R : candidates) {
    bool duplicate = false;
    for (const auto &U : unique) {
      if ((R - U).norm() < 1e-6 || rotationAngle(R, U) < kDedupAngle) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      unique.push_back(R);
    }
  }
  return unique;
}

double GraspExecutor::rotationAngle(const Eigen::Matrix3d &R1,
                                    const Eigen::Matrix3d &R2) {
  const double cos_value = (R1.transpose() * R2).trace() - 1.0;
  return std::acos(std::clamp(cos_value * 0.5, -1.0, 1.0));
}

}  // namespace graspnet_pick
