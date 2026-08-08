from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def navigation_include(nav2_share: Path, namespace: str, use_sim_time, params_file):
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    # navigation_launch.py applies its own namespace; wrapping it in PushRosNamespace
    # would create /amr2/amr2/* and break the AMR2 topics/actions.
    return include


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('hospital_nav2'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_amr1 = LaunchConfiguration('params_amr1')
    params_amr2 = LaunchConfiguration('params_amr2')
    robot_description = (share / 'urdf' / 'amr1_nav.urdf').read_text(encoding='utf-8')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('map', default_value=str(share / 'maps' / 'hospital_map.yaml')),
        DeclareLaunchArgument('params_amr1', default_value=str(share / 'config' / 'nav2_params_amr1.yaml')),
        DeclareLaunchArgument('params_amr2', default_value=str(share / 'config' / 'nav2_params_amr2.yaml')),

        # One shared map server. AMR1 remains exactly on the original stack/topics.
        Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[params_amr1, {'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['map_server'], 'bond_timeout': 0.0}],
        ),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='amr1_robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        ),
        Node(
            package='hospital_nav2', executable='pose_lock_localizer',
            name='pose_lock_localizer', output='screen',
            parameters=[params_amr1, {'use_sim_time': use_sim_time}],
        ),
        navigation_include(nav2_share, '', use_sim_time, params_amr1),
        Node(
            package='hospital_nav2', executable='centerline_navigator',
            name='centerline_navigator', output='screen',
            parameters=[params_amr1, {
                'use_sim_time': use_sim_time,
                # Dual-AMR mode only: the corridor broker keeps the public API unchanged.
                # The planner/controller implementation itself is untouched.
                'goal_topic': '/corridor_priority/amr1/goal',
                'status_topic': '/corridor_priority/amr1/status_raw',
            }],
        ),

        # AMR2 is a namespaced copy of the same AMR1 navigation stack.
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace='amr2', name='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description,
                         'frame_prefix': 'amr2/'}],
            remappings=[('/tf', '/amr2/tf'), ('/tf_static', '/amr2/tf_static')],
        ),
        Node(
            package='hospital_nav2', executable='pose_lock_localizer',
            namespace='amr2', name='pose_lock_localizer', output='screen',
            parameters=[params_amr2, {
                'use_sim_time': use_sim_time,
                'global_frame': 'map',
                'odom_frame': 'amr2/odom',
                'base_frame': 'amr2/base_link',
                'broadcast_hz': 30.0,
                'auto_initial_pose': False,
            }],
            remappings=[
                ('/initialpose', '/amr2/initialpose'),
                ('/initial_pose_locked', '/amr2/initial_pose_locked'),
                ('/tf', '/amr2/tf'),
                ('/tf_static', '/amr2/tf_static'),
            ],
        ),
        Node(
            package='hospital_nav2', executable='world_pose_initializer',
            namespace='amr2', name='world_pose_initializer', output='screen',
            parameters=[{
                'world_pose_topic': '/amr2/world_pose',
                'initialpose_topic': '/amr2/initialpose',
                'lock_topic': '/amr2/initial_pose_locked',
                'frame_id': 'map',
                # Screenshot-confirmed AMR2 map pose. Physical Isaac placement is not changed.
                'fixed_pose_enabled': True,
                'fixed_x': -47.2788,
                'fixed_y': 26.5713,
                'fixed_yaw': 0.0,
            }],
        ),
        navigation_include(nav2_share, 'amr2', use_sim_time, params_amr2),
        Node(
            package='hospital_nav2', executable='centerline_navigator',
            namespace='amr2', name='centerline_navigator', output='screen',
            parameters=[params_amr2, {
                'use_sim_time': use_sim_time,
                'global_frame': 'map',
                'base_frame': 'amr2/base_link',
                'goal_topic': '/corridor_priority/amr2/goal',
                'status_topic': '/corridor_priority/amr2/status_raw',
                'path_topic': '/amr2/centerline_path',
                'cmd_vel_topic': '/amr2/cmd_vel',
                'follow_path_action': '/amr2/follow_path',
                'downsample_factor': 2,
                'robot_safe_radius_m': 0.52,
                'goal_snap_radius_m': 2.0,
                'center_weight': 0.55,
                'turn_penalty': 2.5,
                'path_point_spacing_m': 0.10,
                'corner_stop_sec': 0.15,
                'final_stop_sec': 0.25,
                'rotate_before_segment': True,
                'rotate_max_speed_rad_s': 1.80,
                'rotate_min_speed_rad_s': 0.35,
                'rotate_kp': 2.00,
                'rotate_tolerance_rad': 0.035,
                'rotate_stable_cycles': 2,
                'retry_delay_sec': 0.3,
                'max_follow_path_retries': 0,
                'map_topic': '/map',
                'pose_lock_topic': '/amr2/initial_pose_locked',
            }],
            remappings=[('/tf', '/amr2/tf'), ('/tf_static', '/amr2/tf_static')],
        ),

        # RViz-only AMR2 icon.  It consumes the already-published Isaac world pose
        # and does not publish cmd_vel, TF, initialpose, costmap, or Nav2 actions.
        Node(
            package='hospital_nav2', executable='rviz_robot_icon',
            name='amr2_rviz_icon', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'world_pose_topic': '/amr2/world_pose',
                'marker_topic': '/amr2/rviz_icon',
                'frame_id': 'map',
            }],
        ),

        # Separate corridor coordinator. It only brokers goals/status in dual mode;
        # no Nav2/local-costmap/centerline motion code is replaced.
        Node(
            package='hospital_nav2', executable='corridor_priority_manager',
            name='corridor_priority_manager', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'enabled': True,
                'min_x': -38.5620,
                'max_x': -32.0796,
                'min_y': 10.5645,
                'max_y': 14.2866,
                'path_margin_m': 0.20,
                'exit_margin_m': 0.25,
                'release_delay_sec': 3.0,
                'room_wait_x': -40.0,
                'room_wait_y': 8.0,
                'room_wait_yaw': 1.5707963267948966,
                'elevator_wait_x': -30.0,
                'elevator_wait_y': 18.0,
                'elevator_wait_yaw': -1.5707963267948966,
            }],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2_dual', output='screen',
            arguments=['-d', str(share / 'rviz' / 'dual_navigation.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
