#include "graspnet_pick/executor_node.hpp"

#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
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

std::vector<double> splitDoubles(const std::string &value) {
  std::vector<double> out;
  std::stringstream ss(value);
  std::string item;
  while (std::getline(ss, item, ',')) {
    try {
      out.push_back(std::stod(item));
    } catch (const std::exception &) {
      // ignore malformed item; validateParams will catch size
    }
  }
  return out;
}

template <typename FutureT>
bool waitReady(FutureT &future, double timeout_s) {
  return future.wait_for(std::chrono::duration<double>(timeout_s)) ==
         std::future_status::ready;
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
  set_observation_pose_service_name_ = declare_parameter<std::string>(
      "set_observation_pose_service", "/grasp/set_observation_pose");
  recenter_enabled_ = declare_parameter<bool>("recenter_enabled", true);
  recenter_tolerance_m_ =
      declare_parameter<double>("recenter_tolerance_m", 0.005);
  recenter_max_step_m_ = declare_parameter<double>("recenter_max_step_m", 0.05);
  recenter_max_iterations_ =
      declare_parameter<int>("recenter_max_iterations", 3);
  recenter_settle_s_ = declare_parameter<double>("recenter_settle_s", 0.8);
  gripper_control_mode_ =
      declare_parameter<std::string>("gripper_control_mode", "position");
  gripper_close_squeeze_m_ =
      declare_parameter<double>("gripper_close_squeeze_m", 0.002);
  gripper_force_n_ = declare_parameter<double>("gripper_force_n", 1.5);
  gripper_force_joint_ =
      declare_parameter<std::string>("gripper_force_joint", "gripper");
  gripper_force_topic_ =
      declare_parameter<std::string>("gripper_force_topic", "/control/joint_states");
  gazebo_apply_joint_effort_service_ = declare_parameter<std::string>(
      "gazebo_apply_joint_effort_service", "/apply_joint_effort");
  gazebo_clear_joint_efforts_service_ = declare_parameter<std::string>(
      "gazebo_clear_joint_efforts_service", "/clear_joint_efforts");
  controller_switch_service_ = declare_parameter<std::string>(
      "controller_switch_service", "/controller_manager/switch_controller");
  gripper_controller_name_ =
      declare_parameter<std::string>("gripper_controller_name", "gripper_controller");
  gripper_effort_signs_ = splitDoubles(declare_parameter<std::string>(
      "gripper_effort_signs", "-1.0,1.0"));

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
  gripper_force_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      gripper_force_topic_, rclcpp::QoS(10));
  apply_effort_client_ =
      create_client<gazebo_msgs::srv::ApplyJointEffort>(
          gazebo_apply_joint_effort_service_);
  clear_efforts_client_ = create_client<gazebo_msgs::srv::JointRequest>(
      gazebo_clear_joint_efforts_service_);
  switch_controller_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(
          controller_switch_service_);
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
  set_observation_pose_service_ = create_service<std_srvs::srv::Trigger>(
      set_observation_pose_service_name_,
      std::bind(&GraspExecutor::onSetObservationPose, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

  RCLCPP_INFO(get_logger(),
              "grasp_executor ready: service=%s, observation=%s, motion=%s, "
              "gripper=%s",
              service_name_.c_str(), set_observation_pose_service_name_.c_str(),
              motion_topic_.c_str(), gripper_topic_.c_str());
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
  if (gripper_control_mode_ != "position" && gripper_control_mode_ != "force" &&
      gripper_control_mode_ != "gazebo_force") {
    throw std::invalid_argument(
        "gripper_control_mode must be 'position', 'force' or 'gazebo_force'");
  }
  if (gripper_close_squeeze_m_ < 0.0) {
    throw std::invalid_argument("gripper_close_squeeze_m must be >= 0");
  }
  if (recenter_tolerance_m_ <= 0.0 || recenter_max_step_m_ <= 0.0 ||
      recenter_max_iterations_ < 1 || recenter_settle_s_ < 0.0) {
    throw std::invalid_argument(
        "recenter parameters must be positive (iterations >= 1)");
  }
  if ((gripper_control_mode_ == "force" ||
       gripper_control_mode_ == "gazebo_force") &&
      (gripper_force_n_ < 0.5 || gripper_force_n_ > 3.0)) {
    throw std::invalid_argument(
        "gripper_force_n must be in [0.5, 3.0] N for force control");
  }
  if (gripper_control_mode_ == "gazebo_force") {
    if (gripper_joints_.size() != 2 || gripper_effort_signs_.size() != 2) {
      throw std::invalid_argument(
          "gazebo_force requires two gripper_joints and two gripper_effort_signs");
    }
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
            gripper_joints_.end() ||
        msg->name[i] == gripper_force_joint_) {
      joint_positions_[msg->name[i]] = msg->position[i];
    }
  }
}

void GraspExecutor::onSetObservationPose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (tcp_buffer_.empty()) {
    response->success = false;
    response->message = "no TCP feedback yet";
    return;
  }
  try {
    observation_tcp_ = poseMsgToMatrix(tcp_buffer_.back().pose);
  } catch (const std::exception &exc) {
    response->success = false;
    response->message = std::string("invalid TCP pose: ") + exc.what();
    return;
  }
  response->success = true;
  response->message = "observation pose recorded from current TCP";
  RCLCPP_INFO(get_logger(), "observation pose recorded: p=[%.3f, %.3f, %.3f]",
              (*observation_tcp_)(0, 3), (*observation_tcp_)(1, 3),
              (*observation_tcp_)(2, 3));
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
  std::optional<Eigen::Matrix4d> observation;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    observation = observation_tcp_;
  }
  const double timeout =
      request.timeout_s > 0.0f ? request.timeout_s : motion_timeout_s_;

  if (observation.has_value()) {
    const Eigen::Matrix4d T_obs_ee =
        *observation * invertTransform(T_ee_tcp_);
    if (!moveTo(T_obs_ee, "observation", timeout)) {
      fail(response, ExecuteGrasp::Response::ERROR_MOTION,
           "failed to move to observation pose");
      return;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(0.0, recenter_settle_s_)));
  }

  if (recenter_enabled_) {
    std::string message;
    const uint8_t code = recenterTarget(message);
    if (code != ExecuteGrasp::Response::ERROR_NONE) {
      fail(response, code, message);
      return;
    }
  }

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

  if (!commandGripperDispatch(open_width, "open")) {
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
    commandGripperDispatch(open_width, "safe-open");
    fail(response, ExecuteGrasp::Response::ERROR_MOTION,
         "pregrasp motion failed for all orientation candidates");
    return;
  }

  const Eigen::Matrix4d T_base_ee_grasp =
      *chosen_tcp * invertTransform(T_ee_tcp_);
  if (!moveTo(T_base_ee_grasp, "grasp", timeout)) {
    commandGripperDispatch(open_width, "safe-open");
    fail(response, ExecuteGrasp::Response::ERROR_MOTION,
         "grasp motion failed or timed out");
    return;
  }
  const double close_width =
      std::max(0.0, width - gripper_close_squeeze_m_);
  if (!commandGripperDispatch(close_width, "close")) {
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
    commandGripperDispatch(open_width, "safe-open");
    fail(response, ExecuteGrasp::Response::ERROR_GRIPPER, ss.str());
    return;
  }

  // 先纯竖直抬起，确认没有滑落，再沿抓取反方向后退。
  Eigen::Matrix4d T_base_tcp_lift = *chosen_tcp;
  T_base_tcp_lift(2, 3) += std::max(0.0, lift_height_m_);
  const Eigen::Matrix4d T_base_ee_lift =
      T_base_tcp_lift * invertTransform(T_ee_tcp_);
  if (!moveTo(T_base_ee_lift, "lift", timeout)) {
    fail(response, ExecuteGrasp::Response::ERROR_MOTION,
         "lift motion failed; object may still be held");
    return;
  }
  const auto lift_opening = gripperOpening();
  if (lift_opening.has_value()) {
    RCLCPP_INFO(get_logger(), "gripper opening after lift: %.1fmm",
                *lift_opening * 1000.0);
  }
  if (lift_opening.has_value() && *lift_opening < min_grasp_opening_m_) {
    std::ostringstream ss;
    ss << "object slipped during lift (opening=" << *lift_opening * 1000.0
       << "mm)";
    commandGripperDispatch(open_width, "safe-open");
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

bool GraspExecutor::commandGripperForce(double width_m, double force_n,
                                        const std::string &label) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = {gripper_force_joint_};
  msg.position = {width_m};
  msg.effort = {force_n};
  gripper_force_pub_->publish(msg);
  RCLCPP_INFO(get_logger(),
              "gripper %s (force mode): width=%.1fmm force=%.2fN",
              label.c_str(), width_m * 1000.0, force_n);
  std::this_thread::sleep_for(
      std::chrono::duration<double>(std::max(0.0, gripper_settle_s_)));
  return true;
}

bool GraspExecutor::commandGripperDispatch(double opening_m,
                                           const std::string &label) {
  if (gripper_control_mode_ == "force") {
    return commandGripperForce(opening_m, gripper_force_n_, label);
  }
  if (gripper_control_mode_ == "gazebo_force") {
    if (label == "close") {
      return commandGripperGazeboForce(gripper_force_n_, label);
    }
    return openGripperGazebo(opening_m, label);
  }
  return commandGripper(opening_m, label);
}

uint8_t GraspExecutor::recenterTarget(std::string &message) {
  for (int iter = 0; iter < recenter_max_iterations_; ++iter) {
    std::optional<interfaces::msg::GraspCandidate> candidate;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      candidate = candidate_;
    }
    if (!candidate.has_value()) {
      message = "no GraspCandidate for recentering";
      return ExecuteGrasp::Response::ERROR_NO_CANDIDATE;
    }
    const rclcpp::Time stamp(candidate->header.stamp,
                             get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      message = "candidate has zero timestamp";
      return ExecuteGrasp::Response::ERROR_INVALID_GRASP;
    }
    const auto tcp_match = nearestTcp(stamp);
    if (!tcp_match.has_value()) {
      message = "no TCP feedback for recentering";
      return ExecuteGrasp::Response::ERROR_TRANSFORM;
    }
    const Eigen::Matrix4d T_base_tcp = poseMsgToMatrix(tcp_match->first.pose);
    const Eigen::Matrix4d T_base_cam = T_base_tcp * T_tcp_cam_;
    const Eigen::Vector3d p_cam(candidate->pose.position.x,
                                candidate->pose.position.y,
                                candidate->pose.position.z);
    const Eigen::Vector2d error(p_cam.x(), p_cam.y());
    if (error.norm() <= recenter_tolerance_m_) {
      RCLCPP_INFO(get_logger(),
                  "recenter converged after %d iterations: err=%.1fmm", iter,
                  error.norm() * 1000.0);
      return ExecuteGrasp::Response::ERROR_NONE;
    }

    Eigen::Vector3d delta_cam(p_cam.x(), p_cam.y(), 0.0);
    if (delta_cam.norm() > recenter_max_step_m_) {
      delta_cam *= recenter_max_step_m_ / delta_cam.norm();
    }
    const Eigen::Vector3d delta_base =
        T_base_cam.block<3, 3>(0, 0) * delta_cam;
    Eigen::Matrix4d T_base_tcp_next = T_base_tcp;
    T_base_tcp_next.block<3, 1>(0, 3) += delta_base;
    const Eigen::Matrix4d T_base_ee_next =
        T_base_tcp_next * invertTransform(T_ee_tcp_);
    std::ostringstream label;
    label << "recenter[" << iter << "]";
    RCLCPP_INFO(get_logger(),
                "recenter[%d]: image error=%.1fmm, move camera by "
                "[%.3f, %.3f, 0]m",
                iter, error.norm() * 1000.0, p_cam.x(), p_cam.y());
    if (!moveTo(T_base_ee_next, label.str(), motion_timeout_s_)) {
      message = "recenter motion failed";
      return ExecuteGrasp::Response::ERROR_MOTION;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(0.0, recenter_settle_s_)));
  }
  RCLCPP_WARN(get_logger(),
              "recenter reached max iterations; continue with latest candidate");
  return ExecuteGrasp::Response::ERROR_NONE;
}

bool GraspExecutor::switchGripperController(bool activate) {
  if (!switch_controller_client_) {
    return false;
  }
  if (!switch_controller_client_->service_is_ready() &&
      !switch_controller_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "controller switch service unavailable");
    return false;
  }
  auto request =
      std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (activate) {
    request->activate_controllers = {gripper_controller_name_};
  } else {
    request->deactivate_controllers = {gripper_controller_name_};
  }
  request->strictness =
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->timeout.sec = 2;
  auto future = switch_controller_client_->async_send_request(request);
  if (!waitReady(future, 3.0)) {
    RCLCPP_ERROR(get_logger(), "controller switch timeout");
    return false;
  }
  const bool ok = future.get()->ok;
  if (!ok) {
    RCLCPP_WARN(get_logger(), "controller switch %s failed",
                activate ? "activate" : "deactivate");
  }
  return ok;
}

bool GraspExecutor::applyJointEffort(const std::string &joint, double effort) {
  if (!apply_effort_client_) {
    return false;
  }
  if (!apply_effort_client_->service_is_ready() &&
      !apply_effort_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "Gazebo apply_joint_effort service unavailable");
    return false;
  }
  auto request = std::make_shared<gazebo_msgs::srv::ApplyJointEffort::Request>();
  request->joint_name = joint;
  request->effort = effort;
  request->start_time.sec = 0;
  request->start_time.nanosec = 0;
  request->duration.sec = -1;  // apply continuously
  request->duration.nanosec = 0;
  auto future = apply_effort_client_->async_send_request(request);
  if (!waitReady(future, 3.0)) {
    RCLCPP_ERROR(get_logger(), "apply_joint_effort timeout for %s", joint.c_str());
    return false;
  }
  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(get_logger(), "apply_joint_effort failed for %s: %s",
                 joint.c_str(), response->status_message.c_str());
  }
  return response->success;
}

bool GraspExecutor::clearJointEfforts() {
  if (!clear_efforts_client_) {
    return false;
  }
  if (!clear_efforts_client_->service_is_ready() &&
      !clear_efforts_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(get_logger(), "Gazebo clear_joint_efforts service unavailable");
    return false;
  }
  bool all_ok = true;
  for (const auto &joint : gripper_joints_) {
    auto request = std::make_shared<gazebo_msgs::srv::JointRequest::Request>();
    request->joint_name = joint;
    auto future = clear_efforts_client_->async_send_request(request);
    if (!waitReady(future, 3.0)) {
      RCLCPP_WARN(get_logger(), "clear_joint_efforts timeout for %s", joint.c_str());
      all_ok = false;
      continue;
    }
    future.get();  // response is empty; ready means the request was accepted
  }
  return all_ok;
}

bool GraspExecutor::commandGripperGazeboForce(double force_n,
                                              const std::string &label) {
  if (!switchGripperController(false)) {
    RCLCPP_ERROR(get_logger(),
                 "gripper %s: failed to deactivate position controller; "
                 "constant-force close aborted",
                 label.c_str());
    return false;
  }
  for (std::size_t i = 0; i < gripper_joints_.size(); ++i) {
    const double effort = gripper_effort_signs_[i] * force_n;
    if (!applyJointEffort(gripper_joints_[i], effort)) {
      RCLCPP_ERROR(get_logger(), "gripper %s: constant force failed on %s",
                   label.c_str(), gripper_joints_[i].c_str());
      clearJointEfforts();
      switchGripperController(true);
      return false;
    }
  }
  RCLCPP_INFO(get_logger(),
              "gripper %s (gazebo constant force): force=%.2fN joints=[%s, %s]",
              label.c_str(), force_n, gripper_joints_[0].c_str(),
              gripper_joints_[1].c_str());
  std::this_thread::sleep_for(
      std::chrono::duration<double>(std::max(0.0, gripper_settle_s_)));
  return true;
}

bool GraspExecutor::openGripperGazebo(double width_m, const std::string &label) {
  clearJointEfforts();
  if (!switchGripperController(true)) {
    RCLCPP_ERROR(get_logger(),
                 "gripper %s: failed to reactivate position controller",
                 label.c_str());
    return false;
  }
  return commandGripper(width_m, label);
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
  if (gripper_control_mode_ == "force") {
    const auto it = joint_positions_.find(gripper_force_joint_);
    if (it == joint_positions_.end()) {
      RCLCPP_WARN(get_logger(),
                  "force-mode gripper feedback unavailable; skip verification");
      return std::nullopt;
    }
    return std::abs(it->second);
  }
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

  // 优先“夹爪朝下”：先纯垂直，再向 +X 倾斜 5/10/15/30/45°。
  // 每个倾角先试物体 OBB 的闭合轴，再试当前腕部偏航。
  if (prefer_vertical_approach_) {
    const std::vector<double> tilts_deg{0.0, 5.0, 10.0, 15.0, 30.0, 45.0};
    const std::vector<Eigen::Vector3d> closings{
        R_grasp.col(1), R_current.col(1), R_current.col(0),
        Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)};
    for (std::size_t ti = 0; ti < tilts_deg.size(); ++ti) {
      const double tilt = tilts_deg[ti] * M_PI / 180.0;
      const Eigen::Vector3d approach(std::sin(tilt), 0.0, -std::cos(tilt));
      const std::size_t close_count = ti <= 3 ? 2U : 1U;
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
