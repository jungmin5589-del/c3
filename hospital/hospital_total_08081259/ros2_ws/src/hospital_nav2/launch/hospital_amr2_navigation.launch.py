from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    """AMR2 standalone Nav2 stack with every Nav2 process explicitly namespaced.

    AMR1 remains completely untouched in the root namespace.  AMR2 never launches a
    root /controller_server, /planner_server, /bt_navigator or lifecycle manager, so
    starting AMR2 cannot lifecycle-transition or replace AMR1's Nav2 stack.
    """
    share = Path(get_package_share_directory("hospital_nav2"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    robot_description = (share / "urdf" / "amr1_nav.urdf").read_text(encoding="utf-8")

    # Match the parameter layout to the explicit /amr2 namespace below.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="amr2",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Relative TF destinations are resolved inside namespace=amr2.
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument(
                "map", default_value=str(share / "maps" / "hospital_map_1f.yaml")
            ),
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config" / "nav2_params_amr2.yaml")
            ),

            # Independent AMR2 map server and map lifecycle manager.
            Node(
                package="nav2_map_server",
                executable="map_server",
                namespace="amr2",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_file, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace="amr2",
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
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="amr2",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                        "frame_prefix": "amr2/",
                    }
                ],
                remappings=tf_remaps,
            ),
            Node(
                package="hospital_nav2",
                executable="pose_lock_localizer",
                namespace="amr2",
                name="pose_lock_localizer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "global_frame": "map",
                        "odom_frame": "amr2/odom",
                        "base_frame": "amr2/base_link",
                        "broadcast_hz": 30.0,
                        "auto_initial_pose": True,
                        "initial_x": -47.2788,
                        "initial_y": 26.5713,
                        "initial_yaw": 0.0,
                    }
                ],
                remappings=[
                    ("/initialpose", "/amr2/initialpose"),
                    ("/initial_pose_locked", "/amr2/initial_pose_locked"),
                    ("/tf", "/amr2/tf"),
                    ("/tf_static", "/amr2/tf_static"),
                ],
            ),

            # Nav2 nodes: every process has namespace=amr2 explicitly.
            Node(
                package="nav2_controller",
                executable="controller_server",
                namespace="amr2",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remaps + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                namespace="amr2",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                namespace="amr2",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                namespace="amr2",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                namespace="amr2",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                namespace="amr2",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                namespace="amr2",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remaps
                + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace="amr2",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": lifecycle_nodes,
                        "bond_timeout": 0.0,
                    }
                ],
            ),

            Node(
                package="hospital_nav2",
                executable="centerline_navigator",
                namespace="amr2",
                name="centerline_navigator",
                output="screen",
                parameters=[
                    configured_params,
                    {
                        "use_sim_time": use_sim_time,
                        "global_frame": "map",
                        "base_frame": "amr2/base_link",
                        "goal_topic": "/amr2/center_goal",
                        "status_topic": "/amr2/center_goal/status",
                        "path_topic": "/amr2/centerline_path",
                        "cmd_vel_topic": "/amr2/cmd_vel",
                        "follow_path_action": "/amr2/follow_path",
                        "map_topic": "/amr2/map",
                        "pose_lock_topic": "/amr2/initial_pose_locked",
                        "traffic_pause_topic": "/amr2/traffic_pause",
                    },
                ],
                remappings=[("/tf", "/amr2/tf"), ("/tf_static", "/amr2/tf_static")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace="amr2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(share / "rviz" / "amr2_navigation.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("/tf", "/amr2/tf"), ("/tf_static", "/amr2/tf_static")],
            ),
        ]
    )
