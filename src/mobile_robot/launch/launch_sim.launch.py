from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_path = get_package_share_directory(package_name)
    controller_yaml = os.path.join(pkg_path, "config", 'my_controller.yaml')
    
    # 1. Setup Model Paths
    install_dir = os.path.abspath(os.path.join(pkg_path, '..'))

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[install_dir]
    )

    # 2. Path to the House World
    world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    # 3. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4. Gazebo (passing the controller params here)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + controller_yaml
        }.items()
    )

    # 5. Delayed Spawn Robot (Wait 5 seconds for world to load)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'my_robot', 
                    '-topic', 'robot_description',
                    '-x', '0.0', '-y', '0.1', '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    # 6. Controller Spawners
    omni_drive = Node(
        package="controller_manager",
        executable='spawner',
        arguments=["omni_base_controller"],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable='spawner',
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 7. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/scan'
        }.items()
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp,
        spawn_robot,
        omni_drive,
        joint_state_broadcaster,
        slam_toolbox,
        rviz
    ])