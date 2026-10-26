
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('scara_pole_marker')
    moveit_share = get_package_share_directory(
        'scara_pole_marker_moveit_config'
    )
    gazebo_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(
        description_share, 'urdf', 'scara_robot.urdf.xacro'
    )
    controllers_file = os.path.join(
        moveit_share, 'config', 'ros2_controllers.yaml'
    )

    robot_description = {
        'robot_description': os.popen(
            f'xacro {xacro_file}'
        ).read()
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'scara_robot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])
