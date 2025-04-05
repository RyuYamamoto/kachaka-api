import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.dirname(get_package_share_directory('kachaka_description'))
        ]
    )

    world_file = os.path.join(
        get_package_share_directory('kachaka_gazebo'), 'world', 'world.sdf')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + world_file
        }.items(),
    )

    kachaka_description_path = os.path.join(get_package_share_directory('kachaka_gazebo'))

    xacro_file = os.path.join(get_package_share_directory('kachaka_description'), 'robot', 'kachaka.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_description = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_description}

    rviz_config_file = os.path.join(kachaka_description_path, 'rviz', 'default.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description, '-name', 'kachaka', '-allow_renaming', 'false'],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            '/kachaka/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/kachaka/imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/kachaka/front_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/kachaka/back_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/kachaka/front_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/kachaka/back_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/kachaka/tof_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/kachaka/tof_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
        ],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
        ign_resource_path,
        robot_state_publisher,
        gz_sim,
        gz_spawn_entity,
        bridge,
        rviz,
    ])
