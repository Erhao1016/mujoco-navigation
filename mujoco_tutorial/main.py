import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("model.xml")
data = mujoco.MjData(model)

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