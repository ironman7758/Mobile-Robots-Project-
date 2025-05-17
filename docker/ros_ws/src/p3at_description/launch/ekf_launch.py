from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    ekf_param_file = os.path.join(
        get_package_share_directory('p3at_description'),
        'params',
        'ekf.yaml'
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param_file],
        ),
    ])
