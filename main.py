import mujoco
import mujoco.viewer
import numpy as np
import time

# 加载模型
model = mujoco.MjModel.from_xml_path("model1.xml")
data = mujoco.MjData(model)

# =========任务3验证调试输出=========
print("====场景刚体清单====")
for i in range(model.nbody):
    b_name = model.body(i).name
    px, py, pz = model.body_pos[i]
    print(f"{b_name} | 初始坐标 x:{px:.2f}, y:{py:.2f}")
# ==================================

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 控制指令：
        # ctrl[0] = X方向速度
        # ctrl[1] = Y方向速度
        # ctrl[2] = 绕Z轴旋转速度（角速度）
        
        data.ctrl[0] = 0.5    # X速度
        data.ctrl[1] = 0.0    # Y速度
        data.ctrl[2] = 0.3    # 角速度（顺时针/逆时针转）
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)