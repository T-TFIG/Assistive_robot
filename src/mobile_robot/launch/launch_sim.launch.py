import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('mobile_robot')
    
    models_path = os.path.join(pkg_path, 'models')
    
    pkg_share_path = os.path.abspath(os.path.join(pkg_path, '..'))

    full_resource_path = models_path + ':' + pkg_share_path + ':' + pkg_path


    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=full_resource_path
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=full_resource_path
    )
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    world_file_path = os.path.join(pkg_path, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"-r {world_file_path}"}.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mobile_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 4. The Bridge (Syncs ROS 2 and Gazebo clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # # base Cam
            # '/base_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/base_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/base_cam/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/base_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # # Wrist Cam
            # '/wrist_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/wrist_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/wrist_cam/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/wrist_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # 2D LiDAR Scan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # # IMU sensor
            # '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
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

    arm_controller_spawner = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
                output='screen'
            )
        ]
    )

    imu_broadcaster_spawner = TimerAction(
        period=11.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["imu_sensor_broadcaster"],
                output='screen'
            )
        ]
    )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[
    #         os.path.join(pkg_path, 'config', 'ekf.yaml'),
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )

    # open3d = Node(
    #     package='mobile_robot',
    #     executable='open3d_mapping_node',
    #     name='open3d_mapper',
    #     output='screen'
    # )

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
        arm_controller_spawner,
        imu_broadcaster_spawner,
        # open3d
    ])