import os
import sys
import time


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
# 项目导航核心
# =====================================

from navigation.navigation_core import NavigationCore


class NavigationNode(Node):
    """
    ROS2 导航节点。

    订阅：
        /odom
        nav_msgs/msg/Odometry

    发布：
        /cmd_vel
        geometry_msgs/msg/Twist

    作用：
        1. 接收机器人实时位置
        2. 调用 A* 规划路径
        3. 调用 PD 控制器计算速度
        4. 发布速度命令
    """

    def __init__(
        self,
    ):
        super().__init__(
            "navigation_node"
        )

        # =================================
        # 导航起点和终点
        # =================================

        self.start_world = (
            0.0,
            0.0,
        )

        self.goal_world = (
            -2.0,
            2.0,
        )

        # =================================
        # 创建导航核心
        # =================================

        self.navigation = NavigationCore(
            waypoint_step=4,
            kp=2.0,
            kd=0.1,
            max_speed=0.9,
            reach_threshold=0.08,
        )

        # 提前规划路径
        self.navigation.plan_path(
            start_world=self.start_world,
            goal_world=self.goal_world,
        )

        # =================================
        # 当前机器人状态
        # =================================

        self.current_position = None

        self.last_control_time = (
            time.perf_counter()
        )

        self.goal_message_printed = False

        # =================================
        # 订阅 /odom
        # =================================

        self.odom_subscription = (
            self.create_subscription(
                Odometry,
                "/odom",
                self.odom_callback,
                10,
            )
        )

        # =================================
        # 发布 /cmd_vel
        # =================================

        self.cmd_vel_publisher = (
            self.create_publisher(
                Twist,
                "/cmd_vel",
                10,
            )
        )

        # 100 Hz 控制周期
        self.timer_period = 0.01

        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback,
        )

        self.get_logger().info(
            "Navigation node started."
        )

        self.get_logger().info(
            "Subscribing to /odom"
        )

        self.get_logger().info(
            "Publishing to /cmd_vel"
        )

        self.get_logger().info(
            f"Start: {self.start_world}"
        )

        self.get_logger().info(
            f"Goal: {self.goal_world}"
        )

    # =====================================
    # 接收机器人位置
    # =====================================

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        """
        接收 simulation_node 发布的机器人位置。
        """
        x = float(
            message.pose.pose.position.x
        )

        y = float(
            message.pose.pose.position.y
        )

        self.current_position = (
            x,
            y,
        )

    # =====================================
    # 控制循环
    # =====================================

    def timer_callback(
        self,
    ) -> None:
        """
        周期执行：
            1. 根据 /odom 获取当前位置
            2. 调用 NavigationCore
            3. 发布 /cmd_vel
        """
        if self.current_position is None:
            return

        current_time = (
            time.perf_counter()
        )

        dt = (
            current_time
            - self.last_control_time
        )

        self.last_control_time = (
            current_time
        )

        vx, vy, wz = (
            self.navigation.compute_command(
                current_position=(
                    self.current_position
                ),
                dt=dt,
            )
        )

        self.publish_velocity(
            vx=vx,
            vy=vy,
            wz=wz,
        )

        if (
            self.navigation.is_goal_reached()
            and not self.goal_message_printed
        ):
            self.get_logger().info(
                "Goal reached. "
                "Publishing zero velocity."
            )

            self.goal_message_printed = True

    # =====================================
    # 发布速度命令
    # =====================================

    def publish_velocity(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        """
        将速度发布到 /cmd_vel。
        """
        message = Twist()

        message.linear.x = float(
            vx
        )

        message.linear.y = float(
            vy
        )

        message.linear.z = 0.0

        message.angular.x = 0.0
        message.angular.y = 0.0

        message.angular.z = float(
            wz
        )

        self.cmd_vel_publisher.publish(
            message
        )

    # =====================================
    # 发布停止命令
    # =====================================

    def publish_stop(
        self,
    ) -> None:
        self.publish_velocity(
            vx=0.0,
            vy=0.0,
            wz=0.0,
        )


def main(
    args=None,
) -> None:
    rclpy.init(
        args=args
    )

    node = NavigationNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        node.get_logger().info(
            "Stopping navigation node."
        )

    finally:
        node.publish_stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()