import os
import sys

import mujoco
import numpy as np


# =====================================
# 项目路径
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
# 导入底层机器人控制器
# =====================================

from controller.controller import RobotController


class SimulationCore:
    """
    与 ROS2 无关的 MuJoCo 仿真核心。

    负责：
        1. 加载 MuJoCo 模型
        2. 获取机器人当前位置
        3. 接收速度指令 vx、vy、wz
        4. 通过 RobotController 控制机器人
        5. 推进 MuJoCo 仿真

    以后 ROS2 simulation_node 只需要：
        - 订阅 /cmd_vel
        - 调用 set_velocity()
        - 发布机器人位置到 /odom
    """

    def __init__(
        self,
        model_path: str | None = None,
        robot_body_name: str = "robot",
        initial_position: tuple[
            float,
            float,
        ] | None = None,
    ):
        if model_path is None:
            model_path = os.path.join(
                CURRENT_DIR,
                "world.xml",
            )

        if not os.path.exists(
            model_path
        ):
            raise FileNotFoundError(
                "找不到 MuJoCo 模型文件：\n"
                f"{model_path}"
            )

        self.model_path = model_path
        self.robot_body_name = (
            robot_body_name
        )

        # 加载 MuJoCo 模型
        self.model = (
            mujoco.MjModel.from_xml_path(
                self.model_path
            )
        )

        # 创建运行时数据
        self.data = mujoco.MjData(
            self.model
        )

        if initial_position is not None:
            self.set_robot_pose(
                x=initial_position[0],
                y=initial_position[1],
                yaw=0.0,
                forward=False,
            )

        # 计算初始状态
        mujoco.mj_forward(
            self.model,
            self.data,
        )

        # 获取机器人 body id
        self.robot_body_id = (
            self.model.body(
                self.robot_body_name
            ).id
        )

        # 创建底层控制器
        self.robot_controller = (
            RobotController(
                self.model,
                self.data,
            )
        )

        self.camera_name = "robot_camera"
        self.camera_renderer = None
        self.depth_renderer = None

        # 保存最近一次速度命令
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0

        print(
            f"MuJoCo model loaded: "
            f"{self.model_path}"
        )

        print(
            f"Robot body: "
            f"{self.robot_body_name}"
        )

    # =====================================
    # 设置机器人位姿
    # =====================================

    def set_robot_pose(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        forward: bool = True,
    ) -> None:
        """
        设置机器人初始位姿。

        当前 robot.xml 使用 tx、ty、rz 三个关节：
            tx 控制 x
            ty 控制 y
            rz 控制 yaw
        """
        joint_values = {
            "tx": float(x),
            "ty": float(y),
            "rz": float(yaw),
        }

        for joint_name, value in joint_values.items():
            joint_id = self.model.joint(
                joint_name
            ).id

            qpos_address = self.model.jnt_qposadr[
                joint_id
            ]

            self.data.qpos[
                qpos_address
            ] = value

        if forward:
            mujoco.mj_forward(
                self.model,
                self.data,
            )

    # =====================================
    # 获取机器人位置
    # =====================================

    def get_robot_position(
        self,
    ) -> tuple[float, float]:
        """
        返回机器人当前世界坐标：
            (x, y)
        """
        position = self.data.xpos[
            self.robot_body_id
        ]

        return (
            float(position[0]),
            float(position[1]),
        )

    # =====================================
    # 获取机器人完整三维位置
    # =====================================

    def get_robot_position_3d(
        self,
    ) -> tuple[float, float, float]:
        """
        返回机器人当前世界坐标：
            (x, y, z)
        """
        position = self.data.xpos[
            self.robot_body_id
        ]

        return (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )

    # =====================================
    # 获取机器人朝向
    # =====================================

    def get_robot_yaw(
        self,
    ) -> float:
        """
        返回机器人绕 Z 轴的角度，单位为弧度。
        """
        joint_id = self.model.joint(
            "rz"
        ).id

        qpos_address = self.model.jnt_qposadr[
            joint_id
        ]

        return float(
            self.data.qpos[
                qpos_address
            ]
        )

    def get_robot_pose_2d(
        self,
    ) -> tuple[float, float, float]:
        """
        返回机器人二维位姿：
            (x, y, yaw)
        """
        x, y = self.get_robot_position()
        yaw = self.get_robot_yaw()

        return x, y, yaw

    # =====================================
    # 读取机器人摄像头
    # =====================================

    def get_camera_image(
        self,
        width: int = 320,
        height: int = 240,
    ):
        """
        返回机器人顶部摄像头的 RGB 图像。

        图像格式是 numpy.ndarray，形状为：
            (height, width, 3)
        """
        if self.camera_renderer is None:
            self.camera_renderer = mujoco.Renderer(
                self.model,
                height=height,
                width=width,
            )

        self.camera_renderer.update_scene(
            self.data,
            camera=self.camera_name,
        )

        return self.camera_renderer.render()

    def get_camera_depth(
        self,
        width: int = 160,
        height: int = 120,
    ):
        """
        返回机器人顶部摄像头的深度图。

        图像格式是 numpy.ndarray，形状为：
            (height, width)
        """
        if self.depth_renderer is None:
            self.depth_renderer = mujoco.Renderer(
                self.model,
                height=height,
                width=width,
            )

            self.depth_renderer.enable_depth_rendering()

        self.depth_renderer.update_scene(
            self.data,
            camera=self.camera_name,
        )

        return self.depth_renderer.render()

    # =====================================
    # 激光雷达式射线测距
    # =====================================

    def cast_navigation_rays(
        self,
        relative_angles: list[float],
        max_distance: float = 5.0,
        ray_height: float = 0.22,
    ) -> list[float]:
        """
        从机器人附近向多个方向发射水平射线。

        返回每条射线命中的距离；没有命中则返回 max_distance。
        relative_angles 是相对机器人 yaw 的角度，单位为弧度。
        """
        x, y, yaw = self.get_robot_pose_2d()

        ray_start = np.array(
            [
                x + 0.18 * np.cos(yaw),
                y + 0.18 * np.sin(yaw),
                ray_height,
            ],
            dtype=np.float64,
        )

        geom_group = np.ones(
            6,
            dtype=np.uint8,
        )

        distances = []

        for relative_angle in relative_angles:
            ray_angle = (
                yaw
                + float(relative_angle)
            )

            ray_direction = np.array(
                [
                    np.cos(ray_angle),
                    np.sin(ray_angle),
                    0.0,
                ],
                dtype=np.float64,
            )

            hit_geom_id = np.array(
                [-1],
                dtype=np.int32,
            )

            distance = mujoco.mj_ray(
                self.model,
                self.data,
                ray_start,
                ray_direction,
                geom_group,
                1,
                self.robot_body_id,
                hit_geom_id,
            )

            if distance < 0:
                distance = max_distance

            distances.append(
                min(
                    float(distance),
                    max_distance,
                )
            )

        return distances

    # =====================================
    # 设置速度
    # =====================================

    def set_velocity(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        """
        设置机器人速度。

        参数：
            vx：X方向速度
            vy：Y方向速度
            wz：绕Z轴角速度
        """
        self.last_vx = float(vx)
        self.last_vy = float(vy)
        self.last_wz = float(wz)

        self.robot_controller.set_velocity(
            vx=self.last_vx,
            vy=self.last_vy,
            wz=self.last_wz,
        )

        self.robot_controller.update()

    # =====================================
    # 获取最近速度命令
    # =====================================

    def get_velocity_command(
        self,
    ) -> tuple[float, float, float]:
        """
        返回最近一次发送给机器人的速度：
            (vx, vy, wz)
        """
        return (
            self.last_vx,
            self.last_vy,
            self.last_wz,
        )

    # =====================================
    # 停止机器人
    # =====================================

    def stop(
        self,
    ) -> None:
        """
        停止机器人。
        """
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0

        self.robot_controller.stop()

    # =====================================
    # 推进仿真
    # =====================================

    def step(
        self,
        step_count: int = 1,
    ) -> None:
        """
        推进 MuJoCo 仿真。

        step_count：
            连续推进的仿真步数
        """
        if step_count <= 0:
            raise ValueError(
                "step_count 必须大于 0。"
            )

        for _ in range(
            step_count
        ):
            mujoco.mj_step(
                self.model,
                self.data,
            )

    # =====================================
    # 重置仿真
    # =====================================

    def reset(
        self,
    ) -> None:
        """
        将 MuJoCo 仿真恢复到初始状态。
        """
        mujoco.mj_resetData(
            self.model,
            self.data,
        )

        mujoco.mj_forward(
            self.model,
            self.data,
        )

        self.stop()

    # =====================================
    # 场景信息
    # =====================================

    def print_scene_information(
        self,
    ) -> None:
        """
        输出场景刚体和 actuator 信息。
        """
        print("\n===== Scene Bodies =====")

        for body_id in range(
            self.model.nbody
        ):
            body_name = self.model.body(
                body_id
            ).name

            position = self.model.body_pos[
                body_id
            ]

            print(
                f"{body_name:<20}"
                f"x={position[0]:.2f}, "
                f"y={position[1]:.2f}, "
                f"z={position[2]:.2f}"
            )

        print("\n===== Actuators =====")

        for actuator_id in range(
            self.model.nu
        ):
            actuator_name = (
                self.model.actuator(
                    actuator_id
                ).name
            )

            print(
                actuator_id,
                actuator_name,
            )
