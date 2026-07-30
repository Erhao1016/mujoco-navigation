import mujoco
import numpy as np
from PIL import Image
import yaml
import os



# =====================================
# 路径
# =====================================


CURRENT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)


WORLD_PATH = os.path.join(
    CURRENT_DIR,
    "../simulation/world.xml"
)


OUTPUT_DIR = os.path.join(
    CURRENT_DIR,
    "../maps"
)


if not os.path.exists(OUTPUT_DIR):

    os.makedirs(
        OUTPUT_DIR
    )




# =====================================
# 加载 MuJoCo 世界
# =====================================


model = mujoco.MjModel.from_xml_path(
    WORLD_PATH
)


# 创建数据对象
data = mujoco.MjData(model)


# 计算初始状态
mujoco.mj_forward(
    model,
    data
)



print("========================")
print(" Terrain Map Generator ")
print("========================")




# =====================================
# 地图参数
# =====================================


MAP_SIZE = 400


WORLD_SIZE = 10


WORLD_MIN = -5


RESOLUTION = (
    WORLD_SIZE /
    MAP_SIZE
)



# 空闲区域
# 白色

grid = np.ones(
    (
        MAP_SIZE,
        MAP_SIZE
    ),
    dtype=np.uint8
) * 255




# =====================================
# 坐标转换
# =====================================


def world_to_pixel(x, y):


    px = int(
        (x - WORLD_MIN)
        /
        RESOLUTION
    )


    py = int(
        (y - WORLD_MIN)
        /
        RESOLUTION
    )


    return px, py




# =====================================
# 绘制障碍物
# =====================================


def add_obstacle(
        x,
        y,
        size_x,
        size_y
):


    xmin = x - size_x
    xmax = x + size_x

    ymin = y - size_y
    ymax = y + size_y



    px1, py1 = world_to_pixel(
        xmin,
        ymin
    )


    px2, py2 = world_to_pixel(
        xmax,
        ymax
    )



    px1 = max(
        0,
        px1
    )

    py1 = max(
        0,
        py1
    )


    px2 = min(
        MAP_SIZE-1,
        px2
    )


    py2 = min(
        MAP_SIZE-1,
        py2
    )



    grid[
        py1:py2,
        px1:px2
    ] = 0





# =====================================
# 读取 MuJoCo 障碍物
# =====================================


print("\nDetected obstacles:")



for i in range(
        model.ngeom
):


    geom = model.geom(i)



    # 获取所属body编号
    body_id = int(
        geom.bodyid[0]
    )


    body_name = (
        model.body(body_id)
        .name
    )



    # 跳过地面

    if body_name == "world":

        continue



    # 跳过机器人

    if body_name == "robot":

        continue



    # 只处理box

    if geom.type == mujoco.mjtGeom.mjGEOM_BOX:



        # ===============================
        # 修改点：
        # geom.pos 是局部坐标
        # data.xpos 是世界坐标
        # ===============================

        pos = data.xpos[
            body_id
        ]


        size = geom.size



        print(
            body_name,
            "position:",
            round(float(pos[0]),2),
            round(float(pos[1]),2),
            "size:",
            round(float(size[0]),2),
            round(float(size[1]),2)
        )



        add_obstacle(
            pos[0],
            pos[1],
            size[0],
            size[1]
        )





# =====================================
# 保存 pgm
# =====================================


pgm_path = os.path.join(
    OUTPUT_DIR,
    "terrain_map.pgm"
)



Image.fromarray(
    grid
).save(
    pgm_path
)



print(
    "\nSaved:",
    pgm_path
)




# =====================================
# 保存 Nav2 yaml
# =====================================


yaml_data = {


    "image":
    "terrain_map.pgm",


    "resolution":
    float(RESOLUTION),


    "origin":
    [
        WORLD_MIN,
        WORLD_MIN,
        0
    ],


    "negate":
    0,


    "occupied_thresh":
    0.65,


    "free_thresh":
    0.25

}



yaml_path = os.path.join(
    OUTPUT_DIR,
    "terrain_map.yaml"
)



with open(
        yaml_path,
        "w"
) as f:


    yaml.dump(
        yaml_data,
        f
    )



print(
    "Saved:",
    yaml_path
)


print(
    "\n===== Map generation finished ====="
)