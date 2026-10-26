
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "simple_scara",
            package_name="scara_pole_marker_moveit_config"
        )
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("scara_pole_marker_moveit_config"),
        "config",
        "moveit.rviz"
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )
    


    return LaunchDescription([
        move_group,
        rviz,
        
    ])
