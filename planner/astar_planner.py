import heapq
import math
import numpy as np

#欧氏距离
def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(grid, start, goal):
    """
    A* 路径规划

    grid: 二维数组，0=障碍，255=可通行
    start: (row, col)
    goal: (row, col)
    """

    height, width = grid.shape

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        # 到达终点，回溯路径
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        row, col = current

        # 四连通（上下左右）
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr = row + dr
            nc = col + dc

            # 边界检查
            if not (0 <= nr < height and 0 <= nc < width):
                continue

            # 障碍物检查
            if grid[nr, nc] == 0:
                continue

            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get((nr, nc), float("inf")):
                came_from[(nr, nc)] = current
                g_score[(nr, nc)] = tentative_g
                f_score = tentative_g + heuristic((nr, nc), goal)
                heapq.heappush(open_set, (f_score, (nr, nc)))

    return None  # 无解


def plan_from_corner(grid):
    height, width = grid.shape
    start = (height - 1, 0)      # 左下角
    goal = (0, width - 1)         # 右上角

    return astar(grid, start, goal)