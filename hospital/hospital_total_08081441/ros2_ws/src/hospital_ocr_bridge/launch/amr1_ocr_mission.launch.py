from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("hospital_ocr_bridge"))
    # install/<package>/share/<package> -> project root is five levels up.
    default_project_root = str(package_share.parents[4])
    project_root = LaunchConfiguration("project_root")
    patient = LaunchConfiguration("patient")

    ocr_node = Node(
        package="hospital_ocr_bridge",
        executable="hospital_ocr_node",
        name="amr1_ocr",
        output="screen",
        parameters=[
            str(package_share / "config" / "ocr_params.yaml"),
            {
                "amr_id": "amr1",
                "image_topic": "/amr1/camera/front/color/image_raw",
                "request_topic": "/amr1/ocr/request",
                "result_topic": "/amr1/ocr/result",
                "control_topic": "/amr1/ocr/control",
                "output_root": PathJoinSubstitution([project_root, "output", "ocr"]),
            },
        ],
    )

    mission_manager = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["python3", PathJoinSubstitution([project_root, "patient_transport_manager.py"]), patient, "--amr", "amr1"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("project_root", default_value=default_project_root),
            DeclareLaunchArgument("patient", default_value="1"),
            ocr_node,
            mission_manager,
        ]
    )
