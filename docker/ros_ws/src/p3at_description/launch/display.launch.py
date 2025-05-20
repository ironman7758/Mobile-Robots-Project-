import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("p3at_description")
    description_file = os.path.join(pkg_share, "urdf", "pioneer1.urdf")
    world_file = os.path.join(pkg_share, "world", "empty.sdf")
    robot_description = ParameterValue(
        Command(['xacro ', description_file]),
        value_type=str
    )

    # Launch arguments
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Robot description publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo server (headless) launch
    sdf_world = ExecuteProcess(
        cmd=["gz", "sim", world_file],
        name="create_world",
        output="both"
    )

    # Spawn robot entity
    spawn_entity = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", description_file, "-z", "0.2"],
        name="spawn_robot",
        output="both"
    )

    # Joint state publisher
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # Bridge Gazebo topics to ROS 2
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
          "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
          "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
          "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
          "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry"
        ]
    )

    # EKF localization (publish odom->base_link)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
          os.path.join(pkg_share, 'config', 'ekf.yaml'),
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Static transform: map->odom
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

    # Static transform: odom->base_link (in case EKF TF not sufficient)
    static_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_launch_arg,
        use_sim_time_arg,
        sdf_world,
        spawn_entity,
        robot_state_publisher,
        ros_gz_bridge,
        joint_state_pub,
        robot_localization_node,
        static_map_to_odom,
        static_odom_to_base_link,
        rviz_node,
    ])
