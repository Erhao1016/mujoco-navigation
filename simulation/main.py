import os
import sys
import time

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
from simulation.simulation_core import SimulationCore


# =====================================
# 导航参数
# =====================================

START_WORLD = (
    0.0,
    0.0,
)

GOAL_WORLD = (
    -2.0,
    2.0,
)


# =====================================
# 创建导航核心
# =====================================

navigation = NavigationCore(
    waypoint_step=4,
    kp=2.0,
    kd=0.1,
    max_speed=0.9,
    reach_threshold=0.08,
)


# =====================================
# 创建仿真核心
# =====================================

simulation = SimulationCore()


# =====================================
# 规划路径
# =====================================

navigation.plan_path(
    start_world=START_WORLD,
    goal_world=GOAL_WORLD,
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
    "Simulation started..."
)


last_control_time = time.perf_counter()


with mujoco.viewer.launch_passive(
    simulation.model,
    simulation.data,
) as viewer:

    while viewer.is_running():

        current_position = (
            simulation.get_robot_position()
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
            )
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