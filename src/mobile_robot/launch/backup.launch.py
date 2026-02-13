from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():

    package_name = 'mobile_robot'
    pkg_path = get_package_share_directory(package_name)
    controller_yaml = os.path.join(pkg_path, "config", 'my_controller.yaml')
    install_dir = os.path.abspath(os.path.join(pkg_path, '..'))
    
    # really really important we tell the gazebo where is the model stl path is or else it will continue giving an error
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[install_dir]
    )


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --------------------
    # Gazebo (CORRECT WAY)
    # --------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # --------------------
    # Spawn Robot
    # --------------------
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    # --------------------
    # Controllers (EVENT-BASED)
    # --------------------
    # omni_drive = Node(
    #     package="controller_manager",
    #     executable='spawner',
    #     arguments=["wheel_velocity_controller"]
    # )

    # joint_board_spawner = Node(
    #     package="controller_manager",
    #     executable='spawner',
    #     arguments=["joint_state_broadcaster"]
    # )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp,
        spawn_robot
        # omni_drive,
        # joint_board_spawner
    ])
