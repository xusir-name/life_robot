import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("hand_eye_calibration")
    default_poses = os.path.join(pkg_share, "config", "target_poses.yaml")
    default_output = os.path.expanduser(
        "~/life_robot/config/calibration/hand_eye_calibration.yaml"
    )
    default_extrinsics = os.path.expanduser(
        "~/life_robot/config/calibration/camera_tcp_extrinsics.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("poses_file", default_value=default_poses),
            DeclareLaunchArgument("output_yaml", default_value=default_output),
            DeclareLaunchArgument("extrinsics_yaml", default_value=default_extrinsics),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame"),
            DeclareLaunchArgument("tcp_frame", default_value="tcp"),
            DeclareLaunchArgument("motion_target_frame", default_value="link6"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("tcp_topic", default_value="/arm_tcp_state"),
            DeclareLaunchArgument("motion_topic", default_value="/target_pose"),
            DeclareLaunchArgument(
                "motion_status_topic", default_value="/target_pose_status"
            ),
            DeclareLaunchArgument("marker_dict", default_value="DICT_6X6_250"),
            DeclareLaunchArgument("marker_id", default_value="0"),
            DeclareLaunchArgument("marker_size_m", default_value="0.16"),
            DeclareLaunchArgument("hand_eye_method", default_value="TSAI"),
            DeclareLaunchArgument("min_samples", default_value="6"),
            DeclareLaunchArgument("max_time_diff_s", default_value="0.15"),
            DeclareLaunchArgument("settle_time_s", default_value="1.0"),
            DeclareLaunchArgument("move_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("restore_home", default_value="true"),
            Node(
                package="hand_eye_calibration",
                executable="hand_eye_calibrator",
                name="hand_eye_calibrator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "poses_file": LaunchConfiguration("poses_file"),
                        "output_yaml": LaunchConfiguration("output_yaml"),
                        "extrinsics_yaml": LaunchConfiguration("extrinsics_yaml"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "camera_frame": LaunchConfiguration("camera_frame"),
                        "tcp_frame": LaunchConfiguration("tcp_frame"),
                        "motion_target_frame": LaunchConfiguration(
                            "motion_target_frame"
                        ),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "tcp_topic": LaunchConfiguration("tcp_topic"),
                        "motion_topic": LaunchConfiguration("motion_topic"),
                        "motion_status_topic": LaunchConfiguration(
                            "motion_status_topic"
                        ),
                        "marker_dict": LaunchConfiguration("marker_dict"),
                        "marker_id": ParameterValue(
                            LaunchConfiguration("marker_id"), value_type=int
                        ),
                        "marker_size_m": ParameterValue(
                            LaunchConfiguration("marker_size_m"), value_type=float
                        ),
                        "hand_eye_method": LaunchConfiguration("hand_eye_method"),
                        "min_samples": ParameterValue(
                            LaunchConfiguration("min_samples"), value_type=int
                        ),
                        "max_time_diff_s": ParameterValue(
                            LaunchConfiguration("max_time_diff_s"), value_type=float
                        ),
                        "settle_time_s": ParameterValue(
                            LaunchConfiguration("settle_time_s"), value_type=float
                        ),
                        "move_timeout_s": ParameterValue(
                            LaunchConfiguration("move_timeout_s"), value_type=float
                        ),
                        "restore_home": ParameterValue(
                            LaunchConfiguration("restore_home"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
