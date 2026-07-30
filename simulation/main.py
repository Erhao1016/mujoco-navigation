import mujoco
import mujoco.viewer
import time
import sys
import os


# =====================================
# 添加项目根目录到 Python 路径
# 方便导入 controller
# =====================================

CURRENT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

PROJECT_ROOT = os.path.dirname(
    CURRENT_DIR
)

sys.path.append(PROJECT_ROOT)


from controller.controller import RobotController



# =====================================
# 加载 MuJoCo 世界
# =====================================


MODEL_PATH = os.path.join(
    CURRENT_DIR,
    "world.xml"
)


model = mujoco.MjModel.from_xml_path(
    MODEL_PATH
)


data = mujoco.MjData(
    model
)



# =====================================
# 创建机器人控制器
# =====================================


controller = RobotController(
    model,
    data
)



# =====================================
# Task3:
# 输出场景物体
# =====================================


print("\n==========================")
print(" MuJoCo Navigation Demo ")
print("==========================\n")


print("===== Scene Bodies =====")


for i in range(model.nbody):

    name = model.body(i).name

    pos = model.body_pos[i]


    print(
        f"{name:<20}"
        f"x={pos[0]:.2f}, "
        f"y={pos[1]:.2f}, "
        f"z={pos[2]:.2f}"
    )



print("========================\n")



# =====================================
# Task2:
# 查看速度控制接口
# =====================================


print("===== Actuators =====")


for i in range(model.nu):

    print(
        i,
        model.actuator(i).name
    )


print("====================\n")




# =====================================
# 启动 MuJoCo Viewer
# =====================================


with mujoco.viewer.launch_passive(
        model,
        data
) as viewer:



    print(
        "Simulation started..."
    )


    while viewer.is_running():



        # =================================
        # Task2:
        # 设置机器人速度
        #
        # vx:
        #   X方向速度
        #
        # vy:
        #   Y方向速度
        #
        # wz:
        #   Z轴旋转速度
        # =================================


        controller.set_velocity(
            vx=0.5,
            vy=0.0,
            wz=0.3
        )



        # 写入 MuJoCo actuator

        controller.update()



        # 仿真推进

        mujoco.mj_step(
            model,
            data
        )



        # 更新显示

        viewer.sync()



        # 控制刷新速度

        time.sleep(
            0.01
        )



print(
    "Simulation finished."
)