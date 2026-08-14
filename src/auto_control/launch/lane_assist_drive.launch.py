import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_share = get_package_share_directory("auto_control")
    camera_share = get_package_share_directory("camera_driver")
    vehicle_share = get_package_share_directory("vehicle_config")
    detector_params = os.path.join(auto_share, "config", "front_lane_detector.yaml")
    vesc_params = os.path.join(vehicle_share, "config", "vesc_config.yaml")
    camera_launch = os.path.join(camera_share, "launch", "camera_driver.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("duty", default_value="0.040"),
        DeclareLaunchArgument("virtual_duty", default_value="0.025"),
        DeclareLaunchArgument("max_duty", default_value="0.050"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={"preview_enabled": "false", "publish_enabled": "true"}.items(),
        ),
        Node(
            package="vesc_initializer", executable="vesc_initialize_node",
            name="vesc_initialize_node", output="screen", parameters=[vesc_params],
        ),
        Node(
            package="auto_control", executable="front_lane_detector",
            name="front_lane_detector", output="screen", parameters=[detector_params],
        ),
        Node(
            package="auto_control", executable="lane_assist_drive",
            name="lane_assist_drive", output="screen",
            parameters=[{
                "cruise_duty": LaunchConfiguration("duty"),
                "virtual_cruise_duty": LaunchConfiguration("virtual_duty"),
                "max_duty": LaunchConfiguration("max_duty"),
            }],
        ),
    ])
