import os
from ament_index_python import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration as LC

cal_node_config = os.path.join(
    get_package_share_directory("gyro_cal_rig"),
    "config",
    "cal_node_config.yaml"
)


def launch_gyro_driver(context, **kwargs):
    driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("riptide_gyro"),
                "launch",
                "gyro.launch.py"
            )
        ),
        launch_arguments=[
            ("port", LC("gyro_port"))
        ]
    )
    
    if LC("gyro_driver_disabled").perform(context) == "False":
        return [ driver ]

    return []
    


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gyro_port", default_value="/dev/ttyUSB0", description="Port connected to gyro"),
        DeclareLaunchArgument("rig_port", default_value="/dev/ttyUSB1", description="Port connected to rig"),
        DeclareLaunchArgument("gyro_driver_disabled", default_value="False", description="Disable gyro driver"),
        
        GroupAction([
            PushRosNamespace("cal_rig"),
            
            #gyro driver
            OpaqueFunction(function=launch_gyro_driver),
            
            #cal rig node
            Node(
                package="gyro_cal_rig",
                executable="gyro_rig_node",
                name="gyro_rig_node",
                output="screen",
                parameters=[{
                    "port" : LC("rig_port")
                }]
            ),
            
            #cal control node
            Node(
                package="gyro_cal_rig",
                executable="gyro_calibration_node.py",
                name="gyro_calibration_node",
                output="screen",
                parameters=[
                    cal_node_config
                ]
            )
        ], scoped=True)
    ])