import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('mobile_robot')
    
    # Define the models path clearly
    # This points to: install/mobile_robot/share/mobile_robot/models
    models_path = os.path.join(pkg_path, 'models')
    
    # We also need the share directory itself so Gazebo can resolve package:// URIs
    pkg_share_path = os.path.abspath(os.path.join(pkg_path, '..'))

    # Combine them into a single string
    full_resource_path = models_path + ':' + pkg_share_path

    # Set both variable names to be safe across different versions of Gazebo Sim/Ignition
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=full_resource_path
    )
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=full_resource_path
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    world_file_path = os.path.join(pkg_path, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')

    # 1. Gazebo Sim Launch - Pass gz_args as a single string to avoid parsing errors
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"-r {world_file_path}"}.items()
    )

    # 2. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Spawn Robot (Higher Z to avoid floor clipping)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mobile_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.3'
        ],
        output='screen'
    )

    # 4. The Bridge (Syncs ROS 2 and Gazebo clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 5. Controller Spawners (Delayed for stability)
    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster'], output='screen')
        ]
    )

    omni_drive = TimerAction(
        period=7.0,
        actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['omni_base_controller'], output='screen')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        set_ign_resource_path,
        set_gz_resource_path,
        gazebo,
        bridge,
        rsp,
        spawn_robot,
        joint_state_broadcaster,
        omni_drive,
    ])