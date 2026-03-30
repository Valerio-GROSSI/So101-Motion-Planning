from pathlib import Path
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "so101",
        package_name="so101_robot_moveit"
    ).to_moveit_configs()

    rviz_config_file = str(
        Path(moveit_config.package_path) / "config" / "moveit.rviz"
    )

    # chemin vers OMPL
    ompl_config = os.path.join(
        get_package_share_directory("so101_robot_moveit"),
        "config",
        "ompl_planning.yaml"
    )

    # MoveIt (avec OMPL)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ompl_config  # 👈 AJOUT IMPORTANT
        ],
    )

    # 🎮 RViz (interface)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])