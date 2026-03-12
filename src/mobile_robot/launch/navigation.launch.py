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
        'nav2.rviz'
    )

    nav2_params = os.path.join(pkg_path, "config", "nav2_params.yaml")

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
            'extra_gazebo_args': '--ros-args --params-file ' + controller_yaml
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
                    '-Y', '-0.0'
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': os.path.join(pkg_path, 'maps', 'turtlebot3_house.yaml'),
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False'
        }.items()
    )

    delayed_joint_broadcaster = TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster]
    )

    delayed_omni_drive = TimerAction(
        period=10.0,
        actions=[omni_drive]
    )

    delayed_nav2 = TimerAction(
        period=12.0,  
        actions=[nav2]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp,
        spawn_robot,
        delayed_joint_broadcaster,
        delayed_omni_drive,
        delayed_nav2,
        rviz
    ])