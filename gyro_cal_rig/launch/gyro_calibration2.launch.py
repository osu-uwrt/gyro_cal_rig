import os
from ament_index_python import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("gyro_cal_rig"),
                    "launch",
                    "gyro_procedure.launch.py"
                )
            ),
            launch_arguments=[
                ("gyro_procedure_node", "gyro_calibration2_node")
            ]
        )
    ])
