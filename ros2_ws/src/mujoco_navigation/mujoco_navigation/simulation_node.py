import os
import sys


# =====================================
# 项目根目录
# =====================================

CURRENT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

PROJECT_ROOT = os.path.dirname(
    CURRENT_DIR
)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(
        0,
        PROJECT_ROOT,
    )


# =====================================
# ROS2
# 当前 Mac 没有 ROS2，暂时不能运行
# =====================================

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# =====================================
# 项目模块
# =====================================

from simulation.simulation_core import SimulationCore


class SimulationNode(Node):
    """
    MuJoCo ROS2 仿真节点。

    订阅：
        /cmd_vel
        geometry_msgs/msg/Twist

    发布：
        /odom
        nav_msgs/msg/Odometry

    作用：
        1. 接收导航节点发布的速度命令
        2. 控制 MuJoCo 机器人
        3. 推进物理仿真
        4. 发布机器人实时位置
    """

    def __init__(self):
        super().__init__(
            "mujoco_simulation_node"
        )

        # 创建 MuJoCo 仿真核心
        self.simulation = SimulationCore()

        # 保存最新速度命令
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        # 订阅 /cmd_vel
        self.cmd_vel_subscription = (
            self.create_subscription(
                Twist,
                "/cmd_vel",
                self.cmd_vel_callback,
                10,
            )
        )

        # 发布 /odom
        self.odom_publisher = (
            self.create_publisher(
                Odometry,
                "/odom",
                10,
            )
        )

        # 100 Hz 仿真和发布频率
        self.timer_period = 0.01

        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback,
        )

        self.get_logger().info(
            "MuJoCo simulation node started."
        )

        self.get_logger().info(
            "Subscribing to /cmd_vel"
        )

        self.get_logger().info(
            "Publishing to /odom"
        )

    # =====================================
    # 接收速度命令
    # =====================================

    def cmd_vel_callback(
        self,
        message: Twist,
    ) -> None:
        """
        接收 navigation_node 发布的速度命令。
        """
        self.cmd_vx = float(
            message.linear.x
        )

        self.cmd_vy = float(
            message.linear.y
        )

        self.cmd_wz = float(
            message.angular.z
        )

    # =====================================
    # 仿真循环
    # =====================================

    def timer_callback(
        self,
    ) -> None:
        """
        周期执行：
            1. 设置机器人速度
            2. 推进 MuJoCo
            3. 发布机器人位置
        """
        self.simulation.set_velocity(
            vx=self.cmd_vx,
            vy=self.cmd_vy,
            wz=self.cmd_wz,
        )

        self.simulation.step()

        self.publish_odometry()

    # =====================================
    # 发布机器人状态
    # =====================================

    def publish_odometry(
        self,
    ) -> None:
        """
        将机器人当前位置发布到 /odom。
        """
        x, y, z = (
            self.simulation
            .get_robot_position_3d()
        )

        message = Odometry()

        message.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        message.header.frame_id = "odom"

        message.child_frame_id = (
            "base_link"
        )

        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = z

        # 当前机器人导航主要使用 x、y，
        # 姿态暂时设置为单位四元数
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = 0.0
        message.pose.pose.orientation.w = 1.0

        message.twist.twist.linear.x = (
            self.cmd_vx
        )

        message.twist.twist.linear.y = (
            self.cmd_vy
        )

        message.twist.twist.angular.z = (
            self.cmd_wz
        )

        self.odom_publisher.publish(
            message
        )

    # =====================================
    # 停止
    # =====================================

    def stop_robot(
        self,
    ) -> None:
        self.simulation.stop()


def main(
    args=None,
) -> None:
    rclpy.init(
        args=args
    )

    node = SimulationNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        node.get_logger().info(
            "Stopping simulation node."
        )

    finally:
        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()