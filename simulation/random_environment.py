import math
import os
import random

import numpy as np
import yaml
from PIL import Image

from mapping.astar_planner import astar


CURRENT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

PROJECT_ROOT = os.path.dirname(
    CURRENT_DIR
)

MAP_SIZE = 400
WORLD_MIN = -5.0
WORLD_MAX = 5.0
WORLD_SIZE = WORLD_MAX - WORLD_MIN
RESOLUTION = WORLD_SIZE / MAP_SIZE

ROBOT_RADIUS = 0.15
SAFETY_MARGIN = 0.05
FREE_VALUE = 255
OBSTACLE_VALUE = 0


def world_to_grid(
    x: float,
    y: float,
) -> tuple[int, int]:
    col = int(
        (float(x) - WORLD_MIN)
        / RESOLUTION
    )

    row = int(
        (float(y) - WORLD_MIN)
        / RESOLUTION
    )

    return row, col


def grid_to_world(
    row: int,
    col: int,
) -> tuple[float, float]:
    x = (
        float(col)
        * RESOLUTION
        + WORLD_MIN
    )

    y = (
        float(row)
        * RESOLUTION
        + WORLD_MIN
    )

    return x, y


def clip_pixel(
    value: int,
) -> int:
    return max(
        0,
        min(MAP_SIZE - 1, value),
    )


def add_box_obstacle(
    grid: np.ndarray,
    x: float,
    y: float,
    half_size_x: float,
    half_size_y: float,
) -> None:
    xmin = float(x) - float(half_size_x)
    xmax = float(x) + float(half_size_x)
    ymin = float(y) - float(half_size_y)
    ymax = float(y) + float(half_size_y)

    min_row, min_col = world_to_grid(
        xmin,
        ymin,
    )

    max_row, max_col = world_to_grid(
        xmax,
        ymax,
    )

    min_row = clip_pixel(min_row)
    max_row = clip_pixel(max_row)
    min_col = clip_pixel(min_col)
    max_col = clip_pixel(max_col)

    grid[
        min_row:max_row + 1,
        min_col:max_col + 1,
    ] = OBSTACLE_VALUE


def inflate_obstacles(
    grid: np.ndarray,
    radius_pixels: int,
) -> np.ndarray:
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


def boxes_overlap(
    first: dict,
    second: dict,
    margin: float,
) -> bool:
    return (
        abs(first["x"] - second["x"])
        <= first["half_size_x"] + second["half_size_x"] + margin
        and abs(first["y"] - second["y"])
        <= first["half_size_y"] + second["half_size_y"] + margin
    )


def sample_obstacles(
    count: int,
) -> list[dict]:
    obstacles = []
    colors = [
        "1 0 0 1",
        "0 0 1 1",
        "0 0.8 0.1 1",
        "0.8 0 1 1",
        "1 0.7 0 1",
        "0 0.8 0.8 1",
        "0.8 0.3 0.2 1",
    ]

    attempts = 0

    while (
        len(obstacles) < count
        and attempts < 3000
    ):
        attempts += 1

        half_size_x = random.uniform(
            0.25,
            0.70,
        )

        half_size_y = random.uniform(
            0.25,
            0.70,
        )

        obstacle = {
            "name": f"random_obstacle_{len(obstacles) + 1}",
            "x": random.uniform(
                WORLD_MIN + 0.6,
                WORLD_MAX - 0.6,
            ),
            "y": random.uniform(
                WORLD_MIN + 0.6,
                WORLD_MAX - 0.6,
            ),
            "half_size_x": half_size_x,
            "half_size_y": half_size_y,
            "height": random.uniform(
                0.22,
                0.55,
            ),
            "rgba": colors[
                len(obstacles) % len(colors)
            ],
        }

        near_center = math.hypot(
            obstacle["x"],
            obstacle["y"],
        ) < 0.35

        if near_center:
            continue

        if any(
            boxes_overlap(
                obstacle,
                existing,
                margin=0.12,
            )
            for existing in obstacles
        ):
            continue

        obstacles.append(
            obstacle
        )

    if len(obstacles) != count:
        raise RuntimeError(
            "无法生成足够数量的随机障碍物。"
        )

    return obstacles


def build_grid(
    obstacles: list[dict],
) -> np.ndarray:
    raw_grid = np.full(
        (MAP_SIZE, MAP_SIZE),
        FREE_VALUE,
        dtype=np.uint8,
    )

    for obstacle in obstacles:
        add_box_obstacle(
            raw_grid,
            obstacle["x"],
            obstacle["y"],
            obstacle["half_size_x"],
            obstacle["half_size_y"],
        )

    inflation_pixels = int(
        np.ceil(
            (ROBOT_RADIUS + SAFETY_MARGIN)
            / RESOLUTION
        )
    )

    return inflate_obstacles(
        raw_grid,
        inflation_pixels,
    )


def sample_free_cell(
    grid: np.ndarray,
    border_margin: float = 1.0,
) -> tuple[int, int]:
    free_cells = np.argwhere(
        grid == FREE_VALUE
    )

    if len(free_cells) == 0:
        raise RuntimeError(
            "随机地图没有可通行区域。"
        )

    margin_pixels = int(
        math.ceil(
            border_margin
            / RESOLUTION
        )
    )

    interior_cells = free_cells[
        (
            free_cells[:, 0]
            >= margin_pixels
        )
        & (
            free_cells[:, 0]
            < MAP_SIZE - margin_pixels
        )
        & (
            free_cells[:, 1]
            >= margin_pixels
        )
        & (
            free_cells[:, 1]
            < MAP_SIZE - margin_pixels
        )
    ]

    if len(interior_cells) > 0:
        free_cells = interior_cells

    selected = free_cells[
        random.randrange(
            len(free_cells)
        )
    ]

    return (
        int(selected[0]),
        int(selected[1]),
    )


def sample_start_goal(
    grid: np.ndarray,
    minimum_distance: float = 3.0,
) -> tuple[
    tuple[float, float],
    tuple[float, float],
]:
    for _ in range(
        1000
    ):
        start_grid = sample_free_cell(
            grid
        )

        goal_grid = sample_free_cell(
            grid
        )

        start_world = grid_to_world(
            *start_grid
        )

        goal_world = grid_to_world(
            *goal_grid
        )

        distance = math.hypot(
            goal_world[0] - start_world[0],
            goal_world[1] - start_world[1],
        )

        if distance < minimum_distance:
            continue

        if astar(
            grid,
            start_grid,
            goal_grid,
        ):
            return start_world, goal_world

    raise RuntimeError(
        "无法在随机地图中找到可连通的起点和终点。"
    )


def save_world_xml(
    obstacles: list[dict],
    output_path: str,
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
) -> None:
    obstacle_xml = []
    wall_height = 0.40
    wall_thickness = 0.20
    wall_half_length = (
        WORLD_MAX
        - WORLD_MIN
    ) / 2.0
    wall_center = (
        WORLD_MAX
        + WORLD_MIN
    ) / 2.0

    for obstacle in obstacles:
        height = obstacle["height"]

        obstacle_xml.append(
            f"""
        <body
            name="{obstacle["name"]}"
            pos="{obstacle["x"]:.3f} {obstacle["y"]:.3f} {height:.3f}">

            <geom
                name="{obstacle["name"]}_box"
                type="box"
                size="{obstacle["half_size_x"]:.3f} {obstacle["half_size_y"]:.3f} {height:.3f}"
                rgba="{obstacle["rgba"]}"/>

        </body>
"""
        )

    wall_xml = f"""
        <body
            name="wall_north"
            pos="{wall_center:.3f} {WORLD_MAX + wall_thickness:.3f} {wall_height:.3f}">

            <geom
                name="wall_north_box"
                type="box"
                size="{wall_half_length + wall_thickness:.3f} {wall_thickness:.3f} {wall_height:.3f}"
                rgba="0.15 0.15 0.15 1"/>

        </body>

        <body
            name="wall_south"
            pos="{wall_center:.3f} {WORLD_MIN - wall_thickness:.3f} {wall_height:.3f}">

            <geom
                name="wall_south_box"
                type="box"
                size="{wall_half_length + wall_thickness:.3f} {wall_thickness:.3f} {wall_height:.3f}"
                rgba="0.15 0.15 0.15 1"/>

        </body>

        <body
            name="wall_east"
            pos="{WORLD_MAX + wall_thickness:.3f} {wall_center:.3f} {wall_height:.3f}">

            <geom
                name="wall_east_box"
                type="box"
                size="{wall_thickness:.3f} {wall_half_length + wall_thickness:.3f} {wall_height:.3f}"
                rgba="0.15 0.15 0.15 1"/>

        </body>

        <body
            name="wall_west"
            pos="{WORLD_MIN - wall_thickness:.3f} {wall_center:.3f} {wall_height:.3f}">

            <geom
                name="wall_west_box"
                type="box"
                size="{wall_thickness:.3f} {wall_half_length + wall_thickness:.3f} {wall_height:.3f}"
                rgba="0.15 0.15 0.15 1"/>

        </body>
"""

    marker_xml = f"""
        <body
            name="start_marker"
            pos="{start_world[0]:.3f} {start_world[1]:.3f} 0.025">

            <geom
                name="start_marker_disc"
                type="cylinder"
                size="0.22 0.025"
                rgba="0 1 0 0.65"
                contype="0"
                conaffinity="0"/>

        </body>

        <body
            name="goal_marker"
            pos="{goal_world[0]:.3f} {goal_world[1]:.3f} 0.035">

            <geom
                name="goal_marker_disc"
                type="cylinder"
                size="0.28 0.035"
                rgba="1 1 0 0.75"
                contype="0"
                conaffinity="0"/>

        </body>
"""

    xml = f"""<mujoco model="random_navigation_world">

    <compiler autolimits="true"/>

    <include file="robot.xml"/>

    <asset>

        <texture
            name="grid"
            type="2d"
            builtin="checker"
            width="512"
            height="512"
            rgb1=".15 .15 .15"
            rgb2=".25 .25 .25"/>

        <material
            name="ground"
            texture="grid"
            texrepeat="10 10"/>

        <texture
            name="sky"
            type="skybox"
            builtin="gradient"
            width="256"
            rgb1="0.4 0.6 0.8"
            rgb2="0.1 0.2 0.3"/>

    </asset>

    <worldbody>

        <geom
            name="floor"
            type="plane"
            size="5 5 0.1"
            material="ground"/>

{''.join(obstacle_xml)}
{wall_xml}
{marker_xml}

    </worldbody>

</mujoco>
"""

    with open(
        output_path,
        "w",
        encoding="utf-8",
    ) as xml_file:
        xml_file.write(
            xml
        )


def save_map_files(
    grid: np.ndarray,
    map_path: str,
    yaml_path: str,
) -> None:
    Image.fromarray(
        grid
    ).save(
        map_path
    )

    yaml_data = {
        "image": os.path.basename(
            map_path
        ),
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


def generate_random_environment(
    obstacle_count: int = 16,
) -> dict:
    last_error = None

    for _ in range(
        50
    ):
        try:
            obstacles = sample_obstacles(
                obstacle_count
            )

            grid = build_grid(
                obstacles
            )

            start_world, goal_world = sample_start_goal(
                grid,
                minimum_distance=6.0,
            )

            break

        except RuntimeError as error:
            last_error = error

    else:
        raise RuntimeError(
            "无法生成可通行的随机密集地图。"
        ) from last_error

    world_path = os.path.join(
        CURRENT_DIR,
        "random_world.xml",
    )

    map_path = os.path.join(
        PROJECT_ROOT,
        "maps",
        "random_terrain_map.pgm",
    )

    yaml_path = os.path.join(
        PROJECT_ROOT,
        "maps",
        "random_terrain_map.yaml",
    )

    os.makedirs(
        os.path.dirname(map_path),
        exist_ok=True,
    )

    save_world_xml(
        obstacles,
        world_path,
        start_world,
        goal_world,
    )

    save_map_files(
        grid,
        map_path,
        yaml_path,
    )

    return {
        "world_path": world_path,
        "map_path": map_path,
        "yaml_path": yaml_path,
        "start_world": start_world,
        "goal_world": goal_world,
        "obstacles": obstacles,
    }
