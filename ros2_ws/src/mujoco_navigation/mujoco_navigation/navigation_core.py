import os
import sys

import numpy as np
from PIL import Image


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
# 导入现有模块
# =====================================

from controller.pd_controller import PDController
from mapping.planner.astar_planner import astar


class NavigationCore:
    """
    与 ROS2 无关的导航核心。

    负责：
        1. 读取占据栅格地图
        2. 世界坐标与栅格坐标转换
        3. 调用 A* 规划路径
        4. 管理当前目标路径点
        5. 调用 PD 控制器计算 vx、vy、wz

    以后 ROS2 navigation_node 只需要：
        - 接收 /odom
        - 调用 compute_command()
        - 发布 /cmd_vel
    """

    def __init__(
        self,
        map_path: str | None = None,
        world_min: float = -5.0,
        world_max: float = 5.0,
        map_size: int = 400,
        waypoint_step: int = 4,
        kp: float = 2.0,
        kd: float = 0.1,
        max_speed: float = 0.9,
        reach_threshold: float = 0.08,
    ):
        if map_size <= 0:
            raise ValueError(
                "map_size 必须大于 0。"
            )

        if waypoint_step <= 0:
            raise ValueError(
                "waypoint_step 必须大于 0。"
            )

        if world_max <= world_min:
            raise ValueError(
                "world_max 必须大于 world_min。"
            )

        if map_path is None:
            map_path = os.path.join(
                PROJECT_ROOT,
                "maps",
                "terrain_map.pgm",
            )

        self.map_path = map_path

        self.world_min = float(
            world_min
        )

        self.world_max = float(
            world_max
        )

        self.map_size = int(
            map_size
        )

        self.resolution = (
            self.world_max
            - self.world_min
        ) / self.map_size

        self.waypoint_step = int(
            waypoint_step
        )

        self.pd_controller = PDController(
            kp=kp,
            kd=kd,
            max_speed=max_speed,
            reach_threshold=reach_threshold,
        )

        self.grid: np.ndarray | None = None

        # A* 生成的完整栅格路径
        self.path_grid: list[
            tuple[int, int]
        ] = []

        # 转换后的完整世界坐标路径
        self.path_world: list[
            tuple[float, float]
        ] = []

        # 降采样后真正用于跟踪的路径点
        self.waypoints: list[
            tuple[float, float]
        ] = []

        self.target_index = 0

        self.start_world: tuple[
            float,
            float
        ] | None = None

        self.goal_world: tuple[
            float,
            float
        ] | None = None

        self.goal_reached = False

        self.load_map()

    # =====================================
    # 地图读取
    # =====================================

    def load_map(
        self,
    ) -> np.ndarray:
        """
        读取 terrain_map.pgm。

        地图约定：
            0   = 障碍物
            255 = 可通行区域
        """
        if not os.path.exists(
            self.map_path
        ):
            raise FileNotFoundError(
                "找不到地图文件：\n"
                f"{self.map_path}\n\n"
                "请先运行：\n"
                "python mapping/map_generator.py"
            )

        image = Image.open(
            self.map_path
        ).convert(
            "L"
        )

        grid = np.array(
            image,
            dtype=np.uint8,
        )

        # 统一地图值，防止存在中间灰度
        self.grid = np.where(
            grid < 128,
            0,
            255,
        ).astype(
            np.uint8
        )

        print(
            f"Map loaded: {self.map_path}"
        )

        print(
            f"Map shape: {self.grid.shape}"
        )

        print(
            f"Resolution: "
            f"{self.resolution:.4f} m/pixel"
        )

        return self.grid

    # =====================================
    # 坐标转换
    # =====================================

    def world_to_grid(
        self,
        x: float,
        y: float,
    ) -> tuple[int, int]:
        """
        MuJoCo 世界坐标 (x, y)
        转换为 A* 栅格坐标 (row, col)。

        row 对应 y。
        col 对应 x。
        """
        col = int(
            (
                float(x)
                - self.world_min
            )
            / self.resolution
        )

        row = int(
            (
                float(y)
                - self.world_min
            )
            / self.resolution
        )

        return row, col

    def grid_to_world(
        self,
        row: int,
        col: int,
    ) -> tuple[float, float]:
        """
        A* 栅格坐标 (row, col)
        转换为 MuJoCo 世界坐标 (x, y)。
        """
        x = (
            float(col)
            * self.resolution
            + self.world_min
        )

        y = (
            float(row)
            * self.resolution
            + self.world_min
        )

        return x, y

    # =====================================
    # 地图位置检查
    # =====================================

    def is_grid_position_valid(
        self,
        position: tuple[int, int],
    ) -> bool:
        """
        判断栅格位置是否：
            1. 位于地图范围内
            2. 不在障碍物中
        """
        if self.grid is None:
            raise RuntimeError(
                "地图尚未加载。"
            )

        row, col = position

        height, width = (
            self.grid.shape
        )

        if not (
            0 <= row < height
            and 0 <= col < width
        ):
            return False

        return (
            self.grid[row, col]
            != 0
        )

    # =====================================
    # A* 路径规划
    # =====================================

    def plan_path(
        self,
        start_world: tuple[
            float,
            float
        ],
        goal_world: tuple[
            float,
            float
        ],
    ) -> list[
        tuple[float, float]
    ]:
        """
        规划从 start_world 到 goal_world 的路径。

        返回：
            降采样后的世界坐标路径点列表。
        """
        if self.grid is None:
            self.load_map()

        self.start_world = (
            float(start_world[0]),
            float(start_world[1]),
        )

        self.goal_world = (
            float(goal_world[0]),
            float(goal_world[1]),
        )

        start_grid = self.world_to_grid(
            self.start_world[0],
            self.start_world[1],
        )

        goal_grid = self.world_to_grid(
            self.goal_world[0],
            self.goal_world[1],
        )

        print(
            "\n===== A* Path Planning ====="
        )

        print(
            f"Start world: "
            f"{self.start_world}"
        )

        print(
            f"Goal world: "
            f"{self.goal_world}"
        )

        print(
            f"Start grid: "
            f"{start_grid}"
        )

        print(
            f"Goal grid: "
            f"{goal_grid}"
        )

        if not self.is_grid_position_valid(
            start_grid
        ):
            raise ValueError(
                "起点位于地图外或障碍区域："
                f"{self.start_world}"
            )

        if not self.is_grid_position_valid(
            goal_grid
        ):
            raise ValueError(
                "终点位于地图外或障碍区域："
                f"{self.goal_world}"
            )

        planned_path = astar(
            self.grid,
            start_grid,
            goal_grid,
        )

        if not planned_path:
            raise RuntimeError(
                "A* 未找到可行路径。"
            )

        self.path_grid = list(
            planned_path
        )

        self.path_world = [
            self.grid_to_world(
                row,
                col,
            )
            for row, col
            in self.path_grid
        ]

        # 对路径点降采样
        self.waypoints = self.path_world[
            ::self.waypoint_step
        ]

        # 确保最终目标一定在路径中
        if (
            not self.waypoints
            or self.waypoints[-1]
            != self.path_world[-1]
        ):
            self.waypoints.append(
                self.path_world[-1]
            )

        self.target_index = 0
        self.goal_reached = False

        self.pd_controller.reset()

        print(
            f"A* path points: "
            f"{len(self.path_world)}"
        )

        print(
            f"Tracking waypoints: "
            f"{len(self.waypoints)}"
        )

        print(
            "===== Planning finished =====\n"
        )

        return self.waypoints

    # =====================================
    # 当前目标点
    # =====================================

    def current_target(
        self,
    ) -> tuple[float, float] | None:
        """
        返回当前正在跟踪的路径点。
        """
        if not self.waypoints:
            return None

        if self.target_index >= len(
            self.waypoints
        ):
            return None

        return self.waypoints[
            self.target_index
        ]

    # =====================================
    # 计算速度命令
    # =====================================

    def compute_command(
        self,
        current_position: tuple[
            float,
            float
        ],
        dt: float,
    ) -> tuple[
        float,
        float,
        float,
    ]:
        """
        根据机器人当前位置计算速度命令。

        参数：
            current_position：
                当前机器人世界坐标 (x, y)

            dt：
                控制周期，单位为秒

        返回：
            vx、vy、wz

        以后 ROS2 navigation_node 中：
            current_position 来自 /odom
            返回值发布到 /cmd_vel
        """
        if not self.waypoints:
            return 0.0, 0.0, 0.0

        if self.goal_reached:
            return 0.0, 0.0, 0.0

        current = (
            float(current_position[0]),
            float(current_position[1]),
        )

        target = self.current_target()

        if target is None:
            self.goal_reached = True

            return 0.0, 0.0, 0.0

        # 判断是否到达当前路径点
        if self.pd_controller.reached(
            current,
            target,
        ):
            if (
                self.target_index
                < len(self.waypoints) - 1
            ):
                self.target_index += 1

                target = self.current_target()

                print(
                    f"Waypoint "
                    f"{self.target_index + 1}/"
                    f"{len(self.waypoints)}: "
                    f"{target}"
                )

            else:
                self.goal_reached = True

                final_distance = (
                    self.pd_controller
                    .distance_to_target(
                        current,
                        target,
                    )
                )

                print("\nGoal reached!")

                print(
                    f"Final position: "
                    f"({current[0]:.3f}, "
                    f"{current[1]:.3f})"
                )

                print(
                    f"Final distance: "
                    f"{final_distance:.3f} m"
                )

                return 0.0, 0.0, 0.0

        if target is None:
            return 0.0, 0.0, 0.0

        safe_dt = max(
            0.001,
            min(float(dt), 0.1),
        )

        return (
            self.pd_controller
            .compute_velocity(
                current=current,
                target=target,
                dt=safe_dt,
            )
        )

    # =====================================
    # 导航状态
    # =====================================

    def is_goal_reached(
        self,
    ) -> bool:
        """
        返回机器人是否已经到达终点。
        """
        return self.goal_reached

    def progress(
        self,
    ) -> tuple[int, int]:
        """
        返回：
            当前路径点序号
            总路径点数量
        """
        if not self.waypoints:
            return 0, 0

        return (
            min(
                self.target_index + 1,
                len(self.waypoints),
            ),
            len(self.waypoints),
        )