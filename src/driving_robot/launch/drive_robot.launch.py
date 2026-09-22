import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')

    # starts Gazebo, RViz, the robot_state_publisher and the /cmd_vel bridge
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py'),
        )
    )

    drive_robot_node = Node(
        package='driving_robot',
        executable='drive_robot',
        name='drive_robot',
        output='screen'
    )

    return LaunchDescription([
        simulation_launch,
        drive_robot_node
    ])
