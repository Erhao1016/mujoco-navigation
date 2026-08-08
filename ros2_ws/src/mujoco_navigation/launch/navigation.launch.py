from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    同时启动：

    1. MuJoCo 仿真节点
       - 订阅 /cmd_vel
       - 发布 /odom

    2. 导航节点
       - 订阅 /odom
       - 发布 /cmd_vel
    """

    simulation_node = Node(
        package="mujoco_navigation",
        executable="simulation_node",
        name="mujoco_simulation_node",
        output="screen",
    )

    navigation_node = Node(
        package="mujoco_navigation",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
    )

    return LaunchDescription(
        [
            simulation_node,
            navigation_node,
        ]
    )