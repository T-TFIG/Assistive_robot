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
    
    install_dir = os.path.abspath(os.path.join(pkg_path, '..'))

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[install_dir]
    )

    world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'slam.rviz'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
           'world': world_path,
            'use_sim_time': 'true',
            # 'extra_gazebo_args': '--ros-args --params-file ' + controller_yaml
        }.items()
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'my_robot', 
                    '-topic', 'robot_description',
                    '-x', '0.0', '-y', '0.2', '-z', '0.1',
                    '-Y', '-0.0' # 45 degrees in radians
                ],
                output='screen'
            )
        ]
    )

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'transform_timeout': 0.1,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.5,
                'transform_publish_period': 0.02, # 50Hz
            }
        ]
    )

    delayed_joint_broadcaster = TimerAction(
        period=8.0, # Wait until after the robot is spawned (5s + 3s buffer)
        actions=[joint_state_broadcaster]
    )

    delayed_omni_drive = TimerAction(
        period=10.0, # Wait until broadcaster is up
        actions=[omni_drive]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp,
        spawn_robot,
        delayed_joint_broadcaster,
        delayed_omni_drive,
        slam_toolbox,
        rviz
    ])
