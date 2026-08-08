# MuJoCo Navigation

基于 MuJoCo 的二维机器人导航仿真项目。项目实现了随机场景生成、占据栅格地图生成、A* 全局路径规划、PD waypoint 跟踪控制、MuJoCo 物理仿真，以及面向后续扩展的摄像头/雷达式传感器接口。

当前主运行方式是：

```text
随机生成环境
        ↓
生成随机障碍物、四周实体墙、起点/终点标记
        ↓
生成占据栅格地图
        ↓
A* 规划从起点到终点的路径
        ↓
机器人跟踪 A* waypoint
        ↓
MuJoCo viewer 中观察机器人运动
```

## 项目特点

- 使用 MuJoCo 构建二维移动机器人仿真环境。
- 每次运行自动生成随机障碍物、随机起点和随机终点。
- 使用 A* 在占据栅格地图上规划全局路径。
- 使用 PD 控制器跟踪规划出的 waypoint。
- 圆柱体机器人支持 X/Y 平移和 Z 轴旋转。
- 机器人前方带有方向标记点，便于观察朝向。
- 机器人模型上安装了一个前向摄像头，后续可用于视觉感知实验。
- 随机场景四周有实体墙，防止机器人离开地图。
- 起点和终点会在 MuJoCo 场景中以颜色圆盘标出。
- 项目中保留了 ROS2 节点版本，便于后续迁移到 ROS2 通信架构。

## 当前运行方式

推荐使用你当前的 MuJoCo 虚拟环境运行：

```bash
source mujoco_env/bin/activate
mjpython simulation/main.py
```

如果依赖缺失，可以在虚拟环境中安装：

```bash
pip install mujoco numpy pillow pyyaml matplotlib scipy
```

说明：

- 使用 `mjpython` 是为了确保 MuJoCo viewer 在 macOS 上正常启动。
- `simulation/main.py` 是当前主要入口。
- 每次运行会重新生成随机场景和随机地图。

## 当前主流程

### 1. 随机环境生成

入口文件 [simulation/main.py](simulation/main.py) 会调用 [simulation/random_environment.py](simulation/random_environment.py) 生成随机环境。

随机环境包括：

- 随机数量和尺寸的矩形障碍物。
- 随机起点 `START_WORLD`。
- 随机终点 `GOAL_WORLD`。
- 四周实体边界墙。
- 起点绿色圆盘标记。
- 终点黄色圆盘标记。
- 用于 A* 的占据栅格地图。

生成文件包括：

```text
simulation/random_world.xml
maps/random_terrain_map.pgm
maps/random_terrain_map.yaml
```

这些文件默认只保留最近一次运行生成的版本。

### 2. 地图生成

随机环境生成器会把 MuJoCo 场景中的障碍物转换为二维占据栅格地图。

地图约定：

```text
0   = 障碍物
255 = 可通行区域
```

为了让 A* 规划更安全，障碍物会根据机器人半径和安全距离进行膨胀。这样 A* 可以把机器人近似看成一个点，同时仍然保留避障余量。

### 3. A* 路径规划

A* 算法位于：

```text
mapping/planner/astar_planner.py
```

导航核心位于：

```text
navigation/navigation_core.py
```

`NavigationCore` 负责：

- 读取占据栅格地图。
- 世界坐标和栅格坐标互相转换。
- 调用 A* 规划路径。
- 将完整路径转换为世界坐标。
- 对路径进行 waypoint 降采样。
- 管理当前正在跟踪的 waypoint。

### 4. 路径跟踪控制

路径跟踪控制器位于：

```text
controller/pd_controller.py
```

控制器输入：

```text
当前机器人位置 current = (x, y)
当前目标 waypoint target = (x, y)
控制周期 dt
当前机器人 yaw
```

控制器输出：

```text
vx = 世界坐标 X 方向速度
vy = 世界坐标 Y 方向速度
wz = 绕 Z 轴角速度
```

当前机器人是一个全向二维移动模型，可以直接执行 X/Y 平移速度，同时旋转朝向。

### 5. MuJoCo 仿真执行

MuJoCo 仿真核心位于：

```text
simulation/simulation_core.py
```

`SimulationCore` 负责：

- 加载 MuJoCo XML 场景。
- 初始化机器人位置。
- 获取机器人位置和朝向。
- 设置机器人速度。
- 推进物理仿真。
- 读取机器人摄像头图像和深度图。
- 提供 raycast 式测距接口，便于后续做雷达避障。

## 场景和机器人说明

### 机器人模型

机器人模型文件：

```text
simulation/robot.xml
```

机器人由一个圆柱体表示，带有三个关节：

```text
tx = X 方向滑动关节
ty = Y 方向滑动关节
rz = 绕 Z 轴旋转关节
```

对应 actuator：

```text
vx = 控制 X 方向速度
vy = 控制 Y 方向速度
wz = 控制旋转速度
```

机器人上方还包含：

- `front` site：圆柱体上的小圆点，表示机器人正前方。
- `robot_camera`：安装在机器人前上方的摄像头。

### 起点和终点标记

随机场景会自动加入两个非碰撞标记：

```text
绿色圆盘 = 起点
黄色圆盘 = 终点
```

这些标记只用于观察，不会影响机器人碰撞和路径规划。

### 实体墙

随机场景四周会生成四面实体墙：

```text
wall_north
wall_south
wall_east
wall_west
```

它们是 MuJoCo box 几何体，会参与碰撞，也可以被 raycast 测距检测到。

## 项目目录结构

```text
mujoco-navigation/
├── README.md
├── docker-compose.yml
│
├── controller/
│   ├── controller.py
│   ├── pd_controller.py
│   ├── camera_avoidance.py
│   └── depth_avoidance.py
│
├── mapping/
│   ├── map_generator.py
│   └── planner/
│       └── astar_planner.py
│
├── maps/
│   ├── terrain_map.pgm
│   ├── terrain_map.yaml
│   ├── raw_terrain_map.pgm
│   ├── random_terrain_map.pgm
│   └── random_terrain_map.yaml
│
├── navigation/
│   ├── __init__.py
│   └── navigation_core.py
│
├── simulation/
│   ├── __init__.py
│   ├── main.py
│   ├── random_environment.py
│   ├── robot.xml
│   ├── simulation_core.py
│   ├── world.xml
│   └── random_world.xml
│
├── ros2_nodes/
│   ├── simulation_node.py
│   └── navigation_node.py
│
├── ros2_ws/
│   └── src/
│       └── mujoco_navigation/
│
├── simulation_docker/
│   └── Dockerfile
│
└── navigation_docker/
    └── Dockerfile
```

## 目录详细说明

### controller/

控制相关模块。

```text
controller.py
```

底层 MuJoCo actuator 控制器，负责把 `vx/vy/wz` 写入 `data.ctrl`。

```text
pd_controller.py
```

二维 waypoint 跟踪控制器，根据当前位置和目标点计算速度。

```text
camera_avoidance.py
depth_avoidance.py
```

实验性局部避障控制器。当前主流程已经回到 A* 路径跟踪，这两个文件保留用于后续继续研究摄像头/深度图避障。

### mapping/

地图生成和路径规划相关模块。

```text
map_generator.py
```

从固定 MuJoCo 场景 `simulation/world.xml` 中提取障碍物，并生成 `maps/terrain_map.pgm`。

```text
planner/astar_planner.py
```

A* 路径规划核心实现。

### maps/

保存占据栅格地图和 YAML 配置。

```text
terrain_map.pgm
terrain_map.yaml
```

固定场景地图。

```text
random_terrain_map.pgm
random_terrain_map.yaml
```

最近一次随机场景生成的地图。

### navigation/

非 ROS2 版本的导航核心。

```text
navigation_core.py
```

封装地图读取、坐标转换、A* 路径规划、waypoint 管理和 PD 控制调用。

### simulation/

主仿真模块。

```text
main.py
```

当前主入口。负责生成随机环境、规划 A* 路径、启动 MuJoCo viewer、执行控制循环。

```text
random_environment.py
```

随机生成障碍物、起点、终点、实体墙、地图文件和 MuJoCo XML 场景。

```text
simulation_core.py
```

MuJoCo 仿真核心封装。

```text
robot.xml
```

机器人模型定义。

```text
world.xml
```

固定测试场景。

```text
random_world.xml
```

最近一次运行自动生成的随机场景。

### ros2_nodes/

早期 ROS2 节点版本，包含：

```text
simulation_node.py
navigation_node.py
```

用于将仿真和导航拆成 ROS2 节点。

### ros2_ws/

ROS2 package 工作区版本。包含 `mujoco_navigation` 包、launch 文件、setup 配置等。当前日常运行主要使用顶层脚本版 `simulation/main.py`。

### Docker 目录

```text
simulation_docker/
navigation_docker/
docker-compose.yml
```

用于后续容器化仿真和导航环境。当前主要运行方式仍是本地虚拟环境 + `mjpython`。

## 当前主要参数

在 [simulation/main.py](simulation/main.py) 中：

```python
environment = generate_random_environment(
    obstacle_count=16,
)
```

控制随机障碍物数量。

```python
navigation = NavigationCore(
    waypoint_step=4,
    kp=5.0,
    kd=0.15,
    max_speed=4.0,
    reach_threshold=0.14,
)
```

控制 A* waypoint 跟踪速度和到点阈值。

在 [simulation/robot.xml](simulation/robot.xml) 中：

```xml
ctrlrange="-4.5 4.5"
```

控制机器人 X/Y 方向 actuator 的速度范围。

## 运行时输出说明

运行时会打印：

- 随机 world 文件路径。
- 随机 map 文件路径。
- 障碍物位置和尺寸。
- A* 起点/终点坐标。
- A* 栅格坐标。
- A* 路径点数量。
- waypoint 跟踪进度。
- 到达终点后的最终位置和误差。

## 目前状态

当前推荐展示版本是：

```text
A* 全局路径规划 + PD waypoint 跟踪 + 随机场景 + 起终点可视化 + 实体边界墙
```

实验性功能已经保留：

- 摄像头 RGB 图像读取。
- 摄像头深度图读取。
- raycast 雷达式测距。
- camera/depth 避障控制器。

这些可以作为后续扩展方向，而当前主流程保持稳定、清晰。

## 后续可扩展方向

建议后续继续完善：

- 在 MuJoCo viewer 中可视化 A* 路径线。
- 记录实验结果，例如路径长度、运行时间、是否到达终点、碰撞次数。
- 添加碰撞检测统计。
- 将参数集中到 `config.yaml`。
- 加入路径平滑，让 A* 路径更自然。
- 做运行模式切换，例如：

```text
astar_only
astar_lidar_guard
camera_depth
```

- 整理 ROS2 package，使脚本版和 ROS2 版共享同一套核心逻辑。

## Git 分支说明

当前本地分支是：

```text
yangerhao
```

它跟踪远程分支：

```text
origin/yangerhao
```

如果需要把当前项目提交并推送到该分支，通常流程是：

```bash
git add README.md simulation/main.py simulation/random_environment.py simulation/robot.xml simulation/simulation_core.py navigation/navigation_core.py controller/pd_controller.py
git commit -m "Update MuJoCo navigation demo and documentation"
git push origin yangerhao
```

如果远程分支没有别人新的提交，普通 `git push` 就够了。  
如果你说的“覆盖掉 yangerhao 分支”是指强制覆盖远程分支，需要先确认后再使用 `git push --force-with-lease`，避免误删别人提交。
