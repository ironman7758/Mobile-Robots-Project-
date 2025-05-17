# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    world_file       = os.path.join(pkg_share, "world", "empty.sdf")
    robot_description = ParameterValue(
        Command(['xacro ', description_file]),
        value_type=str
    )

    ### declare the use_sim_time argument ###
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    ### existing nodes ###
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    sdf_world = ExecuteProcess(
        cmd=["gz", "sim", world_file],
        name="create_world",
        output="both"
    )

    spawn_entity = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", description_file, "-z", "0.2"],
        name="spawn robot",
        output="both"
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ### add the robot_localization EKF node ###
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

    return LaunchDescription([
        rviz_launch_arg,
        use_sim_time_arg,
        sdf_world,
        spawn_entity,
        robot_state_publisher,
        ros_gz_bridge,
        joint_state_pub,
        robot_localization_node,
        rviz_node,
    ])


