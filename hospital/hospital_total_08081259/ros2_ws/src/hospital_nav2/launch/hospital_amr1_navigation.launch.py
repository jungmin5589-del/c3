from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("hospital_nav2"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    robot_description = (share / "urdf" / "amr1_nav.urdf").read_text(encoding="utf-8")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "namespace": "",
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("map", default_value=str(share / "maps" / "hospital_map.yaml")),
            DeclareLaunchArgument("params_file", default_value=str(share / "config" / "nav2_params.yaml")),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="amr1_robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"yaml_filename": map_file, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                        "bond_timeout": 0.0,
                    }
                ],
            ),
            Node(
                package="hospital_nav2",
                executable="pose_lock_localizer",
                name="pose_lock_localizer",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time, "traffic_pause_topic": "/traffic_pause"}],
            ),
            navigation,
            Node(
                package="hospital_nav2",
                executable="centerline_navigator",
                name="centerline_navigator",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(share / "rviz" / "center_navigation.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
