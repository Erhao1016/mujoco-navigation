import heapq
import math
import numpy as np
import random

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


def plan_random_start_goal(grid):
    """
    随机选取起点和终点（必须在可通行区域）
    """
    height, width = grid.shape
    
    # 找出所有可通行的点
    free_cells = np.argwhere(grid == 255)
    
    if len(free_cells) < 2:
        return None

    # 随机选两个不同的点
    idx1, idx2 = random.sample(range(len(free_cells)), 2)
    start = tuple(free_cells[idx1])
    goal = tuple(free_cells[idx2])
    
    print(f"Random Start (pixel): {start}")
    print(f"Random Goal  (pixel): {goal}")
    
    return astar(grid, start, goal)