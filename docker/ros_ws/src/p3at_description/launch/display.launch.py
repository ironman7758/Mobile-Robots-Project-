import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("p3at_description")

    # Ensure Gazebo/ign-gazebo can find your package's meshes and models
    mesh_dir = os.path.join(pkg_share, 'meshes')
    world_dir = os.path.join(pkg_share, 'world')
    sdf_dir = pkg_share  # package share contains world, meshes, urdf, etc.

    env_gz = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            pkg_share
        ])
    )
    env_gzclassic = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join([
            os.environ.get('GAZEBO_MODEL_PATH', ''),
            mesh_dir
        ])
    )

    # Paths to files
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

    # Publishers and bridges
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

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

    # Static transforms
    static_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom', output='screen',
        arguments=['0','0','0','0','0','0','1','map','odom']
    )

    static_odom_to_base_link = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='odom_to_base_link', output='screen',
        arguments=['0','0','0','0','0','0','1','odom','base_link']
    )

    laser_broadcaster = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser', output='screen',
        arguments=['0','0','0','0','0','0','1','base_link','laser']
    )

    # Gazebo world and robot spawn
    sdf_world = ExecuteProcess(
        cmd=["gz", "sim", world_file],
        name="create_world",
        output="both"
    )

    spawn_entity = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", description_file, "-z", "0.2"],
        name="spawn_robot",
        output="both"
    )

    # RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        env_gz,          # set GZ_SIM_RESOURCE_PATH
        env_gzclassic,   # set GAZEBO_MODEL_PATH
        rviz_launch_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_pub,
        ros_gz_bridge,
        robot_localization_node,
        static_map_to_odom,
        static_odom_to_base_link,
        laser_broadcaster,
        sdf_world,
        spawn_entity,
        rviz_node,
    ])
