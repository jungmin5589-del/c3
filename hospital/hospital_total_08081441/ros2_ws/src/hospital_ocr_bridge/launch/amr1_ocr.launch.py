from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("hospital_ocr_bridge"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_root", default_value=""),
            Node(
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
                        "output_root": LaunchConfiguration("output_root"),
                    },
                ],
            ),
        ]
    )
