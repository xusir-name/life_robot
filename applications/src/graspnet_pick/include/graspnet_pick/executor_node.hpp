#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "gazebo_msgs/srv/apply_joint_effort.hpp"
#include "gazebo_msgs/srv/joint_request.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/msg/grasp_candidate.hpp"
#include "interfaces/srv/execute_grasp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace graspnet_pick {

class GraspExecutor : public rclcpp::Node {
 public:
  explicit GraspExecutor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  using ExecuteGrasp = interfaces::srv::ExecuteGrasp;

  void onCandidate(const interfaces::msg::GraspCandidate::SharedPtr msg);
  void onTcp(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onMotionStatus(const std_msgs::msg::String::SharedPtr msg);
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onExecute(const std::shared_ptr<ExecuteGrasp::Request> request,
                 std::shared_ptr<ExecuteGrasp::Response> response);

  void execute(const ExecuteGrasp::Request &request, ExecuteGrasp::Response &response);
  void fail(ExecuteGrasp::Response &response, uint8_t code,
            const std::string &message);
  Eigen::Matrix4d lookupEeTcp();
  std::optional<std::pair<geometry_msgs::msg::PoseStamped, double>> nearestTcp(
      const rclcpp::Time &target);
  std::optional<std::string> waitMotionDone(double timeout_s);
  bool moveTo(const Eigen::Matrix4d &T_base_ee, const std::string &label,
              double timeout_s);
  bool commandGripper(double opening_m, const std::string &label);
  bool commandGripperForce(double width_m, double force_n,
                           const std::string &label);
  bool commandGripperDispatch(double opening_m, const std::string &label);
  bool commandGripperGazeboForce(double force_n, const std::string &label);
  bool openGripperGazebo(double width_m, const std::string &label);
  bool switchGripperController(bool activate);
  bool applyJointEffort(const std::string &joint, double effort);
  bool clearJointEfforts();
  std::optional<double> gripperOpening();
  std::vector<Eigen::Matrix3d> orientationCandidates(
      const Eigen::Matrix3d &R_grasp, const Eigen::Matrix3d &R_current);
  static std::optional<Eigen::Matrix3d> rotationWithApproach(
      const Eigen::Vector3d &approach, const Eigen::Vector3d &closing);
  static double rotationAngle(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2);
  void validateParams() const;

  // topics / frames
  std::string extrinsics_yaml_;
  std::string candidate_topic_;
  std::string service_name_;
  std::string base_frame_;
  std::string ee_frame_;
  std::string camera_frame_;
  std::string tcp_topic_;
  std::string motion_topic_;
  std::string motion_status_topic_;
  std::string gripper_topic_;
  std::vector<std::string> gripper_joints_;
  std::string joint_states_topic_;
  std::string gripper_control_mode_;
  double gripper_close_squeeze_m_{0.002};
  double gripper_force_n_{1.5};
  std::string gripper_force_joint_;
  std::string gripper_force_topic_;
  std::string gazebo_apply_joint_effort_service_;
  std::string gazebo_clear_joint_efforts_service_;
  std::string controller_switch_service_;
  std::string gripper_controller_name_;
  std::vector<double> gripper_effort_signs_;

  // execution parameters
  double max_candidate_age_s_{3.0};
  double max_stamp_diff_s_{0.15};
  double motion_timeout_s_{30.0};
  double tf_timeout_s_{10.0};
  double settle_time_s_{0.4};
  double finger_length_m_{0.0};
  double pregrasp_distance_m_{0.08};
  double retreat_distance_m_{0.08};
  double lift_height_m_{0.05};
  double gripper_open_m_{0.08};
  double gripper_margin_m_{0.004};
  double gripper_max_opening_m_{0.095};
  double gripper_joint_limit_m_{0.05};
  double gripper_move_time_s_{1.0};
  double gripper_settle_s_{1.5};
  double min_grasp_opening_m_{0.005};
  bool prefer_vertical_approach_{true};

  // state
  Eigen::Matrix4d T_tcp_cam_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_ee_tcp_{Eigen::Matrix4d::Identity()};
  std::string extrinsics_parent_;
  std::string extrinsics_child_;
  std::optional<interfaces::msg::GraspCandidate> candidate_;
  std::deque<geometry_msgs::msg::PoseStamped> tcp_buffer_;
  std::string motion_status_;
  bool has_motion_status_{false};
  std::map<std::string, double> joint_positions_;
  bool busy_{false};
  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr motion_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_force_pub_;
  rclcpp::Client<gazebo_msgs::srv::ApplyJointEffort>::SharedPtr apply_effort_client_;
  rclcpp::Client<gazebo_msgs::srv::JointRequest>::SharedPtr clear_efforts_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;
  rclcpp::Subscription<interfaces::msg::GraspCandidate>::SharedPtr candidate_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Service<ExecuteGrasp>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace graspnet_pick
