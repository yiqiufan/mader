"""Start one native MADER planner and optionally its ideal tracker."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    vehicle_id = LaunchConfiguration("vehicle_id")
    params = LaunchConfiguration("params_file")
    defaults = [
        DeclareLaunchArgument("namespace", default_value="SQ01s"),
        DeclareLaunchArgument("vehicle_id", default_value="1"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("mader_native"), "mader.yaml"]),
        ),
        DeclareLaunchArgument("with_tracker", default_value="false"),
        DeclareLaunchArgument("x", default_value="0"),
        DeclareLaunchArgument("y", default_value="0"),
        DeclareLaunchArgument("z", default_value="1"),
        DeclareLaunchArgument("yaw", default_value="0"),
    ]
    planner = Node(
        package="mader_native",
        executable="mader_handoff_node",
        namespace=namespace,
        name="mader",
        parameters=[params, {"vehicle_id": ParameterValue(vehicle_id, value_type=int)}],
        output="screen",
    )
    tracker = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindPackagePrefix("mader_native"), "lib", "mader_native", "perfect_tracker.py"]),
            namespace, LaunchConfiguration("x"), LaunchConfiguration("y"),
            LaunchConfiguration("z"), LaunchConfiguration("yaw"),
        ],
        condition=IfCondition(LaunchConfiguration("with_tracker")),
        output="screen",
    )
    return LaunchDescription(defaults + [planner, tracker])
