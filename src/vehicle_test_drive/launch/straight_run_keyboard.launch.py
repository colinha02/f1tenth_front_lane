from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_config"), "config", "vesc_config.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("target_erpm", default_value="2500"),
            Node(
                package="vesc_initializer",
                executable="vesc_initialize_node",
                name="vesc_initialize_node",
                output="screen",
                parameters=[vesc_config, {"port": LaunchConfiguration("vesc_port")}],
            ),
            Node(
                package="vehicle_test_drive",
                executable="straight_run_keyboard",
                name="straight_run_keyboard",
                output="screen",
                parameters=[{"target_erpm": LaunchConfiguration("target_erpm")}],
            ),
        ]
    )
