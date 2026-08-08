import os
import sys
import time
import math

import mujoco.viewer


# =====================================
# 添加项目根目录到 Python 路径
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


from navigation.navigation_core import NavigationCore
from simulation.random_environment import generate_random_environment
from simulation.simulation_core import SimulationCore


RAY_ANGLES = [
    -math.pi / 2.0
    + index * math.pi / 30.0
    for index in range(31)
]


def build_lidar_status(
    distances: list[float],
) -> dict:
    right_distances = [
        distance
        for angle, distance
        in zip(RAY_ANGLES, distances)
        if angle < -0.35
    ]

    center_distances = [
        distance
        for angle, distance
        in zip(RAY_ANGLES, distances)
        if -0.35 <= angle <= 0.35
    ]

    left_distances = [
        distance
        for angle, distance
        in zip(RAY_ANGLES, distances)
        if angle > 0.35
    ]

    return {
        "right": min(right_distances),
        "center": min(center_distances),
        "left": min(left_distances),
    }


def robot_to_world_velocity(
    forward: float,
    left: float,
    yaw: float,
) -> tuple[float, float]:
    cos_yaw = math.cos(
        yaw
    )

    sin_yaw = math.sin(
        yaw
    )

    return (
        cos_yaw * forward
        - sin_yaw * left,
        sin_yaw * forward
        + cos_yaw * left,
    )


def apply_lidar_collision_guard(
    vx: float,
    vy: float,
    wz: float,
    current_yaw: float,
    lidar_status: dict,
) -> tuple[float, float, float]:
    """
    只做局部防碰撞，不改变 A* 路线。

    A* 输出的速度仍是主命令；雷达只在非常近时介入。
    """
    center_distance = lidar_status[
        "center"
    ]

    left_distance = lidar_status[
        "left"
    ]

    right_distance = lidar_status[
        "right"
    ]

    if center_distance < 0.28:
        turn_direction = (
            1.0
            if left_distance > right_distance
            else -1.0
        )

        return (
            0.0,
            0.0,
            1.4 * turn_direction,
        )

    guarded_vx = vx
    guarded_vy = vy
    guarded_wz = wz

    if center_distance < 0.45:
        guarded_vx *= 0.35
        guarded_vy *= 0.35

    if left_distance < 0.18:
        push_vx, push_vy = robot_to_world_velocity(
            forward=0.0,
            left=-0.35,
            yaw=current_yaw,
        )

        guarded_vx += push_vx
        guarded_vy += push_vy

    if right_distance < 0.18:
        push_vx, push_vy = robot_to_world_velocity(
            forward=0.0,
            left=0.35,
            yaw=current_yaw,
        )

        guarded_vx += push_vx
        guarded_vy += push_vy

    return guarded_vx, guarded_vy, guarded_wz


# =====================================
# 随机生成环境
# =====================================

environment = generate_random_environment(
    obstacle_count=16,
)

START_WORLD = environment[
    "start_world"
]

GOAL_WORLD = environment[
    "goal_world"
]

WORLD_PATH = environment[
    "world_path"
]

MAP_PATH = environment[
    "map_path"
]

OBSTACLES = environment[
    "obstacles"
]

print("\n===== Random Environment =====")

print(
    f"World: {WORLD_PATH}"
)

print(
    f"Map: {MAP_PATH}"
)

for obstacle in OBSTACLES:
    print(
        f"{obstacle['name']:<20}"
        f"position=({obstacle['x']:.2f}, "
        f"{obstacle['y']:.2f})  "
        f"half_size=({obstacle['half_size_x']:.2f}, "
        f"{obstacle['half_size_y']:.2f})"
    )

print(
    "==============================\n"
)


# =====================================
# 创建 A* 导航核心
# =====================================

navigation = NavigationCore(
    map_path=MAP_PATH,
    waypoint_step=4,
    kp=5.0,
    kd=0.15,
    max_speed=4.0,
    reach_threshold=0.14,
)

navigation.plan_path(
    start_world=START_WORLD,
    goal_world=GOAL_WORLD,
)


# =====================================
# 创建仿真核心
# =====================================

simulation = SimulationCore(
    model_path=WORLD_PATH,
    initial_position=START_WORLD,
)


# =====================================
# 输出场景信息
# =====================================

simulation.print_scene_information()


# =====================================
# 启动仿真
# =====================================

print("\n==========================")
print(" MuJoCo Navigation Demo")
print("==========================")

print(
    f"Start: {START_WORLD}"
)

print(
    f"Goal: {GOAL_WORLD}"
)

print(
    "Simulation started in A* path-following mode with lidar collision guard..."
)


last_control_time = time.perf_counter()


with mujoco.viewer.launch_passive(
    simulation.model,
    simulation.data,
) as viewer:

    while viewer.is_running():

        current_x, current_y, current_yaw = (
            simulation.get_robot_pose_2d()
        )

        current_position = (
            current_x,
            current_y,
        )

        current_time = time.perf_counter()

        dt = (
            current_time
            - last_control_time
        )

        last_control_time = current_time

        vx, vy, wz = (
            navigation.compute_command(
                current_position=current_position,
                dt=dt,
                current_yaw=current_yaw,
            )
        )

        lidar_distances = simulation.cast_navigation_rays(
            relative_angles=RAY_ANGLES,
            max_distance=5.0,
        )

        lidar_status = build_lidar_status(
            lidar_distances
        )

        vx, vy, wz = apply_lidar_collision_guard(
            vx=vx,
            vy=vy,
            wz=wz,
            current_yaw=current_yaw,
            lidar_status=lidar_status,
        )

        simulation.set_velocity(
            vx=vx,
            vy=vy,
            wz=wz,
        )

        simulation.step()

        viewer.sync()

        if navigation.is_goal_reached():
            simulation.stop()

        time.sleep(
            0.01
        )


simulation.stop()

print(
    "Simulation finished."
)
