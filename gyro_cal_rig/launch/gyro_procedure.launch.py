import os
from ament_index_python import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration as LC

RIG_STEPS_PER_REV = 577803.965671

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
                "gyro_bench.launch.py"
            )
        ),
        launch_arguments=[
            ("robot", LC("robot")),
            ("gyro_port", LC("gyro_port")),
        ]
    )
    
    if LC("gyro_driver_disabled").perform(context) == "False":
        return [ driver ]

    return []

def launch_gyro_rig_gui(context, **kwargs):
    gui = Node(
        package="gyro_cal_gui",
        executable="GyroCalGui",
        name="gyro_cal_rig_gui",
        output="screen",
        parameters=[{
            "steps_per_revolution" : RIG_STEPS_PER_REV
        }],
        on_exit=Shutdown()
    )
    
    if LC("cal_gui_disabled").perform(context) == "False":
        return [ gui ]

    return []


def launch_procedure_node(context, **kwargs):
    program_name = LC("gyro_procedure_node").perform(context)
    
    #procedure node
    cal_node = Node(
        package="gyro_cal_rig",
        executable=f"{program_name}.py",
        name="gyro_procedure_node",
        output="screen",
        parameters=[
            cal_node_config,
            {
                "steps_per_revolution" : RIG_STEPS_PER_REV
            }
        ]
    )
    
    return [ cal_node ]
    


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="tempest", description="name of the robot. needed for nav"),
        DeclareLaunchArgument("gyro_procedure_node", default_value="gyro_calibration_node", description="Name of the calibrator/validator program to run."),
        DeclareLaunchArgument("gyro_port", default_value="/dev/ttyUSB0", description="Port connected to gyro"),
        DeclareLaunchArgument("rig_port", default_value="/dev/ttyUSB1", description="Port connected to rig"),
        DeclareLaunchArgument("gyro_driver_disabled", default_value="False", description="Disable gyro driver"),
        DeclareLaunchArgument("cal_gui_disabled", default_value="False", description="Disable calibration rig GUI"),
        
        #gyro driver
        OpaqueFunction(function=launch_gyro_driver),
        
        GroupAction([
            PushRosNamespace(LC("robot")),
            
            #rig gui
            OpaqueFunction(function=launch_gyro_rig_gui),
            
            #procedure node
            OpaqueFunction(function=launch_procedure_node),
            
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
            
        ], scoped=True)
    ])