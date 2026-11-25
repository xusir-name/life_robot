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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("poses_file", default_value=default_poses),
            DeclareLaunchArgument("output_yaml", default_value=default_output),
            DeclareLaunchArgument("marker_dict", default_value="DICT_6X6_250"),
            DeclareLaunchArgument("marker_id", default_value="0"),
            DeclareLaunchArgument("marker_size_m", default_value="0.16"),
            DeclareLaunchArgument("hand_eye_method", default_value="TSAI"),
            DeclareLaunchArgument("min_samples", default_value="6"),
            DeclareLaunchArgument("max_time_diff_s", default_value="0.15"),
            DeclareLaunchArgument("settle_time_s", default_value="1.0"),
            DeclareLaunchArgument("move_timeout_s", default_value="30.0"),
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
                    }
                ],
            ),
        ]
    )
