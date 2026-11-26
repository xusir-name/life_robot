import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _i(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _b(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    default_extrinsics = os.path.expanduser(
        "~/life_robot/config/calibration/camera_tcp_extrinsics.yaml"
    )
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # camera / target box
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument(
            "depth_topic", default_value="/camera/depth/image_raw"
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/camera_info"
        ),
        DeclareLaunchArgument("box_topic", default_value="/grasp/target_box"),
        DeclareLaunchArgument(
            "candidate_topic", default_value="/grasp/best_grasp"
        ),
        DeclareLaunchArgument("pose_topic", default_value="/grasp/best_pose"),
        DeclareLaunchArgument(
            "camera_frame", default_value="camera_optical_frame"
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        # estimation
        DeclareLaunchArgument("backend", default_value="geometric"),
        DeclareLaunchArgument("depth_scale_m", default_value="0.001"),
        DeclareLaunchArgument("stride", default_value="2"),
        DeclareLaunchArgument("min_depth_m", default_value="0.05"),
        DeclareLaunchArgument("max_depth_m", default_value="3.0"),
        DeclareLaunchArgument("max_pair_dt_s", default_value="0.15"),
        DeclareLaunchArgument("sensor_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("box_timeout_s", default_value="10.0"),
        DeclareLaunchArgument("estimate_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("remove_plane", default_value="true"),
        DeclareLaunchArgument("plane_distance_m", default_value="0.012"),
        DeclareLaunchArgument("plane_min_ratio", default_value="0.25"),
        DeclareLaunchArgument("plane_min_points", default_value="80"),
        DeclareLaunchArgument("plane_iterations", default_value="200"),
        DeclareLaunchArgument("plane_max_normal_angle_rad", default_value="0.436"),
        DeclareLaunchArgument("voxel_m", default_value="0.003"),
        DeclareLaunchArgument("cluster_eps_m", default_value="0.008"),
        DeclareLaunchArgument("cluster_min_points", default_value="20"),
        DeclareLaunchArgument("min_width_m", default_value="0.005"),
        DeclareLaunchArgument("max_width_m", default_value="0.09"),
        DeclareLaunchArgument("preferred_width_m", default_value="0.04"),
        DeclareLaunchArgument("nms_translation_m", default_value="0.02"),
        DeclareLaunchArgument("nms_rotation_rad", default_value="0.5236"),
        DeclareLaunchArgument("max_grasps", default_value="200"),
        DeclareLaunchArgument("min_score", default_value="0.25"),
        # execution / hand-eye
        DeclareLaunchArgument("extrinsics_yaml", default_value=default_extrinsics),
        DeclareLaunchArgument("ee_frame", default_value="link6"),
        DeclareLaunchArgument("tcp_topic", default_value="/arm_tcp_state"),
        DeclareLaunchArgument("motion_topic", default_value="/target_pose"),
        DeclareLaunchArgument(
            "motion_status_topic", default_value="/target_pose_status"
        ),
        DeclareLaunchArgument(
            "gripper_topic",
            default_value="/gripper_controller/joint_trajectory",
        ),
        DeclareLaunchArgument(
            "gripper_joints", default_value="gripper_joint1,gripper_joint2"
        ),
        DeclareLaunchArgument("service_name", default_value="/grasp/execute"),
        DeclareLaunchArgument("max_candidate_age_s", default_value="3.0"),
        DeclareLaunchArgument("max_stamp_diff_s", default_value="0.15"),
        DeclareLaunchArgument("motion_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("tf_timeout_s", default_value="10.0"),
        DeclareLaunchArgument("settle_time_s", default_value="0.4"),
        DeclareLaunchArgument("finger_length_m", default_value="0.0"),
        DeclareLaunchArgument("grasp_height_offset_m", default_value="0.0"),
        DeclareLaunchArgument("pregrasp_distance_m", default_value="0.08"),
        DeclareLaunchArgument("retreat_distance_m", default_value="0.08"),
        DeclareLaunchArgument("lift_height_m", default_value="0.05"),
        DeclareLaunchArgument("gripper_open_m", default_value="0.08"),
        DeclareLaunchArgument("gripper_margin_m", default_value="0.004"),
        DeclareLaunchArgument("gripper_max_opening_m", default_value="0.095"),
        DeclareLaunchArgument("gripper_joint_limit_m", default_value="0.05"),
        DeclareLaunchArgument("gripper_move_time_s", default_value="1.0"),
        DeclareLaunchArgument("gripper_settle_s", default_value="1.5"),
        DeclareLaunchArgument("min_grasp_opening_m", default_value="0.005"),
        DeclareLaunchArgument("prefer_vertical_approach", default_value="true"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("gripper_control_mode", default_value="position"),
        DeclareLaunchArgument("gripper_close_squeeze_m", default_value="0.002"),
        DeclareLaunchArgument("gripper_force_n", default_value="1.5"),
        DeclareLaunchArgument("gripper_force_joint", default_value="gripper"),
        DeclareLaunchArgument(
            "gripper_force_topic", default_value="/control/joint_states"
        ),
        DeclareLaunchArgument("gripper_effort_signs", default_value="-1.0,1.0"),
        DeclareLaunchArgument(
            "gazebo_apply_joint_effort_service", default_value="/apply_joint_effort"
        ),
        DeclareLaunchArgument(
            "gazebo_clear_joint_efforts_service", default_value="/clear_joint_efforts"
        ),
        DeclareLaunchArgument(
            "controller_switch_service",
            default_value="/controller_manager/switch_controller",
        ),
        DeclareLaunchArgument(
            "gripper_controller_name", default_value="gripper_controller"
        ),
    ]

    estimator = Node(
        package="graspnet_pick",
        executable="grasp_estimator",
        name="grasp_estimator",
        output="screen",
        parameters=[
            {
                "use_sim_time": _b("use_sim_time"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "box_topic": LaunchConfiguration("box_topic"),
                "candidate_topic": LaunchConfiguration("candidate_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "backend": LaunchConfiguration("backend"),
                "depth_scale_m": _f("depth_scale_m"),
                "stride": _i("stride"),
                "min_depth_m": _f("min_depth_m"),
                "max_depth_m": _f("max_depth_m"),
                "max_pair_dt_s": _f("max_pair_dt_s"),
                "sensor_timeout_s": _f("sensor_timeout_s"),
                "box_timeout_s": _f("box_timeout_s"),
                "estimate_rate_hz": _f("estimate_rate_hz"),
                "remove_plane": _b("remove_plane"),
                "plane_distance_m": _f("plane_distance_m"),
                "plane_min_ratio": _f("plane_min_ratio"),
                "plane_min_points": _i("plane_min_points"),
                "plane_iterations": _i("plane_iterations"),
                "plane_max_normal_angle_rad": _f("plane_max_normal_angle_rad"),
                "voxel_m": _f("voxel_m"),
                "cluster_eps_m": _f("cluster_eps_m"),
                "cluster_min_points": _i("cluster_min_points"),
                "min_width_m": _f("min_width_m"),
                "max_width_m": _f("max_width_m"),
                "preferred_width_m": _f("preferred_width_m"),
                "nms_translation_m": _f("nms_translation_m"),
                "nms_rotation_rad": _f("nms_rotation_rad"),
                "max_grasps": _i("max_grasps"),
                "min_score": _f("min_score"),
            }
        ],
    )

    executor = Node(
        package="graspnet_pick",
        executable="grasp_executor",
        name="grasp_executor",
        output="screen",
        parameters=[
            {
                "use_sim_time": _b("use_sim_time"),
                "extrinsics_yaml": LaunchConfiguration("extrinsics_yaml"),
                "candidate_topic": LaunchConfiguration("candidate_topic"),
                "service_name": LaunchConfiguration("service_name"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "tcp_topic": LaunchConfiguration("tcp_topic"),
                "motion_topic": LaunchConfiguration("motion_topic"),
                "motion_status_topic": LaunchConfiguration("motion_status_topic"),
                "gripper_topic": LaunchConfiguration("gripper_topic"),
                "gripper_joints": LaunchConfiguration("gripper_joints"),
                "max_candidate_age_s": _f("max_candidate_age_s"),
                "max_stamp_diff_s": _f("max_stamp_diff_s"),
                "motion_timeout_s": _f("motion_timeout_s"),
                "tf_timeout_s": _f("tf_timeout_s"),
                "settle_time_s": _f("settle_time_s"),
                "finger_length_m": _f("finger_length_m"),
                "grasp_height_offset_m": _f("grasp_height_offset_m"),
                "pregrasp_distance_m": _f("pregrasp_distance_m"),
                "retreat_distance_m": _f("retreat_distance_m"),
                "lift_height_m": _f("lift_height_m"),
                "gripper_open_m": _f("gripper_open_m"),
                "gripper_margin_m": _f("gripper_margin_m"),
                "gripper_max_opening_m": _f("gripper_max_opening_m"),
                "gripper_joint_limit_m": _f("gripper_joint_limit_m"),
                "gripper_move_time_s": _f("gripper_move_time_s"),
                "gripper_settle_s": _f("gripper_settle_s"),
                "min_grasp_opening_m": _f("min_grasp_opening_m"),
                "prefer_vertical_approach": _b("prefer_vertical_approach"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "gripper_control_mode": LaunchConfiguration("gripper_control_mode"),
                "gripper_close_squeeze_m": _f("gripper_close_squeeze_m"),
                "gripper_force_n": _f("gripper_force_n"),
                "gripper_force_joint": LaunchConfiguration("gripper_force_joint"),
                "gripper_force_topic": LaunchConfiguration("gripper_force_topic"),
                "gripper_effort_signs": LaunchConfiguration("gripper_effort_signs"),
                "gazebo_apply_joint_effort_service": LaunchConfiguration(
                    "gazebo_apply_joint_effort_service"
                ),
                "gazebo_clear_joint_efforts_service": LaunchConfiguration(
                    "gazebo_clear_joint_efforts_service"
                ),
                "controller_switch_service": LaunchConfiguration(
                    "controller_switch_service"
                ),
                "gripper_controller_name": LaunchConfiguration(
                    "gripper_controller_name"
                ),
            }
        ],
    )

    return LaunchDescription(args + [estimator, executor])
