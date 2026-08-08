import os

import mujoco
import numpy as np
import yaml
from PIL import Image


# =====================================
# 文件路径
# =====================================

CURRENT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

PROJECT_ROOT = os.path.dirname(
    CURRENT_DIR
)

WORLD_PATH = os.path.join(
    PROJECT_ROOT,
    "simulation",
    "world.xml",
)

OUTPUT_DIR = os.path.join(
    PROJECT_ROOT,
    "maps",
)

os.makedirs(
    OUTPUT_DIR,
    exist_ok=True,
)


# =====================================
# 地图参数
# =====================================

MAP_SIZE = 400

WORLD_MIN = -5.0
WORLD_MAX = 5.0
WORLD_SIZE = WORLD_MAX - WORLD_MIN

# 每个像素对应的实际距离，单位为米/像素
RESOLUTION = WORLD_SIZE / MAP_SIZE

# robot.xml 中圆柱机器人的半径为 0.15 米
ROBOT_RADIUS = 0.15

# 额外安全距离
SAFETY_MARGIN = 0.05

# 地图像素值
FREE_VALUE = 255
OBSTACLE_VALUE = 0


# =====================================
# 加载 MuJoCo 场景
# =====================================

model = mujoco.MjModel.from_xml_path(
    WORLD_PATH
)

data = mujoco.MjData(
    model
)

# 计算场景中各个 body、geom 的世界坐标
mujoco.mj_forward(
    model,
    data,
)


print("========================")
print(" Terrain Map Generator")
print("========================")

print(f"World file: {WORLD_PATH}")
print(f"Map size: {MAP_SIZE} x {MAP_SIZE}")
print(f"Resolution: {RESOLUTION:.4f} m/pixel")


# =====================================
# 创建原始栅格地图
# =====================================

raw_grid = np.full(
    (MAP_SIZE, MAP_SIZE),
    FREE_VALUE,
    dtype=np.uint8,
)


# =====================================
# 坐标转换
# =====================================

def world_to_pixel(
    x: float,
    y: float,
) -> tuple[int, int]:
    """
    将 MuJoCo 世界坐标转换为地图像素坐标。

    返回：
        px：地图列坐标
        py：地图行坐标
    """
    px = int(
        (float(x) - WORLD_MIN)
        / RESOLUTION
    )

    py = int(
        (float(y) - WORLD_MIN)
        / RESOLUTION
    )

    return px, py


def clip_pixel(
    value: int,
) -> int:
    """
    将像素坐标限制在地图范围内。
    """
    return max(
        0,
        min(MAP_SIZE - 1, value),
    )


# =====================================
# 将矩形障碍物画入地图
# =====================================

def add_box_obstacle(
    grid: np.ndarray,
    x: float,
    y: float,
    half_size_x: float,
    half_size_y: float,
) -> None:
    """
    将 MuJoCo box geom 转换成地图中的矩形障碍物。

    MuJoCo box 的 size 是半尺寸：
        实际宽度 = 2 * half_size_x
        实际长度 = 2 * half_size_y
    """
    xmin = float(x) - float(half_size_x)
    xmax = float(x) + float(half_size_x)

    ymin = float(y) - float(half_size_y)
    ymax = float(y) + float(half_size_y)

    px_min, py_min = world_to_pixel(
        xmin,
        ymin,
    )

    px_max, py_max = world_to_pixel(
        xmax,
        ymax,
    )

    px_min = clip_pixel(px_min)
    px_max = clip_pixel(px_max)

    py_min = clip_pixel(py_min)
    py_max = clip_pixel(py_max)

    # 切片右边界不包含，因此加 1
    grid[
        py_min:py_max + 1,
        px_min:px_max + 1,
    ] = OBSTACLE_VALUE


# =====================================
# 障碍物膨胀
# =====================================

def inflate_obstacles(
    grid: np.ndarray,
    radius_pixels: int,
) -> np.ndarray:
    """
    以圆形结构将障碍物向外膨胀。

    膨胀距离：
        机器人半径 + 安全距离

    这样 A* 可以继续把机器人看成一个点，
    但规划出来的路径会给机器人保留足够空间。
    """
    if radius_pixels <= 0:
        return grid.copy()

    inflated_grid = grid.copy()

    obstacle_rows, obstacle_cols = np.where(
        grid == OBSTACLE_VALUE
    )

    offsets = []

    for row_offset in range(
        -radius_pixels,
        radius_pixels + 1,
    ):
        for col_offset in range(
            -radius_pixels,
            radius_pixels + 1,
        ):
            if (
                row_offset * row_offset
                + col_offset * col_offset
                <= radius_pixels * radius_pixels
            ):
                offsets.append(
                    (
                        row_offset,
                        col_offset,
                    )
                )

    for row, col in zip(
        obstacle_rows,
        obstacle_cols,
    ):
        for row_offset, col_offset in offsets:
            new_row = row + row_offset
            new_col = col + col_offset

            if (
                0 <= new_row < MAP_SIZE
                and 0 <= new_col < MAP_SIZE
            ):
                inflated_grid[
                    new_row,
                    new_col,
                ] = OBSTACLE_VALUE

    return inflated_grid


# =====================================
# 提取 MuJoCo 障碍物
# =====================================

print("\nDetected obstacles:")

obstacle_count = 0

for geom_id in range(
    model.ngeom
):
    geom = model.geom(
        geom_id
    )

    body_id = int(
        geom.bodyid[0]
    )

    body_name = model.body(
        body_id
    ).name

    # 跳过地面和机器人
    if body_name in {
        "world",
        "robot",
    }:
        continue

    # 当前地图生成器只处理 box 障碍物
    if (
        geom.type
        != mujoco.mjtGeom.mjGEOM_BOX
    ):
        continue

    # geom_xpos 是 geom 自身的世界坐标，
    # 比直接使用 geom.pos 更准确
    geom_world_pos = data.geom_xpos[
        geom_id
    ]

    geom_size = geom.size

    x = float(
        geom_world_pos[0]
    )

    y = float(
        geom_world_pos[1]
    )

    half_size_x = float(
        geom_size[0]
    )

    half_size_y = float(
        geom_size[1]
    )

    print(
        f"{body_name:<20}"
        f"position=({x:.2f}, {y:.2f})  "
        f"half_size=({half_size_x:.2f}, "
        f"{half_size_y:.2f})"
    )

    add_box_obstacle(
        raw_grid,
        x,
        y,
        half_size_x,
        half_size_y,
    )

    obstacle_count += 1


if obstacle_count == 0:
    print("Warning: no box obstacles detected.")


# =====================================
# 障碍物膨胀
# =====================================

inflation_radius = (
    ROBOT_RADIUS
    + SAFETY_MARGIN
)

inflation_pixels = int(
    np.ceil(
        inflation_radius
        / RESOLUTION
    )
)

print("\n===== Obstacle Inflation =====")

print(
    f"Robot radius: {ROBOT_RADIUS:.2f} m"
)

print(
    f"Safety margin: {SAFETY_MARGIN:.2f} m"
)

print(
    f"Inflation radius: "
    f"{inflation_radius:.2f} m"
)

print(
    f"Inflation pixels: "
    f"{inflation_pixels}"
)

inflated_grid = inflate_obstacles(
    raw_grid,
    inflation_pixels,
)


# =====================================
# 保存地图
# =====================================

# 未膨胀地图：方便检查真实障碍物大小
raw_map_path = os.path.join(
    OUTPUT_DIR,
    "raw_terrain_map.pgm",
)

Image.fromarray(
    raw_grid
).save(
    raw_map_path
)

# 膨胀后的地图：A* 实际使用
terrain_map_path = os.path.join(
    OUTPUT_DIR,
    "terrain_map.pgm",
)

Image.fromarray(
    inflated_grid
).save(
    terrain_map_path
)


# =====================================
# 保存 Nav2 YAML
# =====================================

yaml_data = {
    "image": "terrain_map.pgm",
    "resolution": float(
        RESOLUTION
    ),
    "origin": [
        WORLD_MIN,
        WORLD_MIN,
        0.0,
    ],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25,
}

yaml_path = os.path.join(
    OUTPUT_DIR,
    "terrain_map.yaml",
)

with open(
    yaml_path,
    "w",
    encoding="utf-8",
) as yaml_file:
    yaml.safe_dump(
        yaml_data,
        yaml_file,
        sort_keys=False,
    )


# =====================================
# 输出结果
# =====================================

raw_obstacle_pixels = int(
    np.sum(
        raw_grid == OBSTACLE_VALUE
    )
)

inflated_obstacle_pixels = int(
    np.sum(
        inflated_grid
        == OBSTACLE_VALUE
    )
)

print("\n===== Map Result =====")

print(
    f"Detected box obstacles: "
    f"{obstacle_count}"
)

print(
    f"Raw obstacle pixels: "
    f"{raw_obstacle_pixels}"
)

print(
    f"Inflated obstacle pixels: "
    f"{inflated_obstacle_pixels}"
)

print(
    f"Saved raw map: "
    f"{raw_map_path}"
)

print(
    f"Saved planning map: "
    f"{terrain_map_path}"
)

print(
    f"Saved YAML: "
    f"{yaml_path}"
)

print(
    "\n===== Map generation finished ====="
)