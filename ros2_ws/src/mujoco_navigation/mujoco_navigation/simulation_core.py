import os
import sys

import mujoco


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