"""Autonomous lane assist using the experimental rotated-window detector.

The stable ``lane_assist_drive.launch.py`` remains unchanged.  This launch
uses the same model and VESC topics, so the existing lane-assist controller can
be evaluated with the experimental detector without any duplicate nodes.
"""

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
    detector_params = os.path.join(
        auto_share, "config", "front_lane_rotated_detector.yaml"
    )
    vesc_params = os.path.join(vehicle_share, "config", "vesc_config.yaml")
    camera_launch = os.path.join(camera_share, "launch", "camera_driver.launch.py")

    return LaunchDescription([
        # Baseline was 0.050.  Keep a small speed reduction in a detected
        # curve: straight = 1.20x (0.060), full curve = 1.10x (0.055).
        DeclareLaunchArgument("duty", default_value="0.060"),
        DeclareLaunchArgument("virtual_duty", default_value="0.060"),
        DeclareLaunchArgument("max_duty", default_value="0.065"),
        DeclareLaunchArgument("minimum_drive_duty", default_value="0.040"),
        DeclareLaunchArgument("startup_duty", default_value="0.060"),
        DeclareLaunchArgument("startup_boost_sec", default_value="0.80"),
        DeclareLaunchArgument("curve_duty_scale_at_full", default_value="0.9167"),
        DeclareLaunchArgument("far_heading_weight", default_value="0.30"),
        DeclareLaunchArgument("steering_sign", default_value="1.0"),
        DeclareLaunchArgument("max_steering", default_value="0.65"),
        DeclareLaunchArgument("short_loss_hold_sec", default_value="0.80"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={"preview_enabled": "false", "publish_enabled": "true"}.items(),
        ),
        Node(
            package="vesc_initializer",
            executable="vesc_initialize_node",
            name="vesc_initialize_node",
            output="screen",
            parameters=[vesc_params],
        ),
        Node(
            package="auto_control",
            executable="front_lane_rotated_detector",
            name="front_lane_rotated_detector",
            output="screen",
            parameters=[detector_params],
        ),
        Node(
            package="auto_control",
            executable="lane_assist_drive",
            name="lane_assist_drive",
            output="screen",
            parameters=[{
                "cruise_duty": LaunchConfiguration("duty"),
                "virtual_cruise_duty": LaunchConfiguration("virtual_duty"),
                "max_duty": LaunchConfiguration("max_duty"),
                "minimum_drive_duty": LaunchConfiguration("minimum_drive_duty"),
                "startup_duty": LaunchConfiguration("startup_duty"),
                "startup_boost_sec": LaunchConfiguration("startup_boost_sec"),
                "curve_duty_scale_at_full": LaunchConfiguration("curve_duty_scale_at_full"),
                "far_heading_weight": LaunchConfiguration("far_heading_weight"),
                "steering_sign": LaunchConfiguration("steering_sign"),
                "max_steering": LaunchConfiguration("max_steering"),
                "short_loss_hold_sec": LaunchConfiguration("short_loss_hold_sec"),
            }],
        ),
    ])
