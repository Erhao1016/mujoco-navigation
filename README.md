# MuJoCo Navigation

基于 MuJoCo 的二维移动机器人导航仿真项目，实现了从**随机场景生成**、**占据栅格地图**、**A\* 全局路径规划**、**PD 路径跟踪**，到 **ROS2 通信** 与 **Docker 容器化**的完整导航链路。

## 这个项目是什么（What is this project?）

一个机器人导航课程/演示项目，包含：

- 一个可在 X/Y 方向平移、绕 Z 轴旋转的**圆柱机器人**（MuJoCo 模型）。
- 场景中随机生成的**矩形障碍物**、实体边界墙、起点/终点标记。
- 从 MuJoCo 场景自动生成的**占据栅格地图**。
- **A\* 全局路径规划**：随机起点 → 随机终点，在地图上规划全局路径。
- **PD 控制器**按位置跟踪路径点，输出 `vx / vy / wz` 速度。
- **雷达式 raycast 测距**与摄像头接口（实验性扩展）。
- **ROS2 版本**：仿真节点与导航节点分离，通过 `/odom`、`/cmd_vel` 话题通信。
- **Docker 版本**：仿真容器与导航容器通过 ROS2 在虚拟网络中交换数据。

### 主流程

```text
随机生成环境（障碍物 + 起终点 + 实体墙）
        ↓
生成占据栅格地图（障碍物按机器人半径膨胀）
        ↓
A* 规划起点 → 终点的全局路径
        ↓
机器人跟踪 A* waypoint（PD 控制 vx/vy/wz）
        ↓
MuJoCo viewer 观察 / ROS2 话题 / Docker 容器
```

### 运行方式对比

| 方式 | 说明 | 适用场景 |
|------|------|---------|
| 本地仿真 | 单进程跑 `simulation/main.py`，MuJoCo viewer 可视化 | 日常演示、调试 |
| ROS2 | 仿真与导航拆成两个节点，通过话题通信 | 学习 ROS2 通信架构 |
| Docker | 两个独立容器通过 ROS2 通信，免安装环境 | 部署、多人复现 |

## 项目架构（Architecture）

### 数据流

```text
   simulation/random_environment.py         controller/pd_controller.py
        │ 随机障碍物/地图/XML                        ▲
        ▼                                           │
   mapping/astar_planner.py (A*)              navigation/navigation_core.py
        │ 全局路径                                 │ waypoint + PD
        ▼                                           │
   navigation/navigation_core.py  ──(vx,vy,wz)──►  仿真执行
        │                                           │
        └──────(当前位置)──────►  simulation/simulation_core.py
                                          │
                                          ▼
                        MuJoCo 物理仿真 / ROS2 / Docker
```

ROS2 版本将这条链路拆为两个节点：

```text
simulation_node（仿真）                     navigation_node（导航）
  订阅 /cmd_vel  ◄────────────────────────  发布 /cmd_vel
  发布 /odom    ────────────────────────►  订阅 /odom
```

Docker 版本把两个节点分别放进独立容器，通过 `ros_network` 虚拟网络交换 ROS2 消息。

### 目录结构

```text
mujoco-navigation/
├── README.md                          # 本文件
├── docker-compose.yml                 # Docker 容器编排（仿真 + 导航）
│
├── controller/                        # 控制器
│   ├── controller.py                  #   底层 MuJoCo actuator 速度控制器
│   ├── pd_controller.py               #   二维 waypoint PD 跟踪控制器
│   ├── camera_avoidance.py            #   实验性摄像头避障
│   └── depth_avoidance.py             #   实验性深度图避障
│
├── mapping/                           # 地图与全局路径规划
│   ├── map_generator.py               #   从固定场景 world.xml 生成 terrain_map.pgm
│   └── astar_planner.py               #   A* 核心实现
│
├── maps/                              # 占据栅格地图（固定场景）
│   ├── terrain_map.pgm
│   └── terrain_map.yaml
│
├── navigation/                        # 导航核心（非 ROS2）
│   └── navigation_core.py             #   地图读取/坐标转换/A*/waypoint/PD 调度
│
├── simulation/                        # 仿真
│   ├── main.py                        #   主入口：随机环境 + A* + viewer + 控制循环
│   ├── random_environment.py          #   随机障碍物/起终点/墙/地图/XML 生成
│   ├── simulation_core.py             #   MuJoCo 仿真核心封装（速度/位姿/传感器）
│   ├── robot.xml                      #   机器人模型（圆柱，tx/ty/rz + vx/vy/wz）
│   └── world.xml                      #   固定测试场景（4 个障碍物）
│
├── ros2_ws/                           # ROS2 工作区
│   └── src/mujoco_navigation/
│       ├── launch/navigation.launch.py
│       ├── mujoco_navigation/
│       │   ├── simulation_node.py     #   订阅 /cmd_vel，发布 /odom
│       │   └── navigation_node.py     #   订阅 /odom，发布 /cmd_vel
│       ├── package.xml
│       └── setup.py
│
├── simulation_docker/                 # 仿真容器 Dockerfile（Ubuntu 24.04 + ROS2 jazzy + mujoco）
└── navigation_docker/                 # 导航容器 Dockerfile（Ubuntu 24.04 + ROS2 jazzy）
```

> 说明：ROS2 节点与顶层脚本共享同一套核心逻辑（`controller/`、`mapping/`、`navigation/`、`simulation/`），通过 `PYTHONPATH` 指向项目根目录导入，避免重复维护。

## 如何运行（How to run this code?）

### 依赖

```bash
pip install mujoco numpy pillow pyyaml matplotlib scipy
```

ROS2 / Docker 方式还需要系统安装 ROS2（如 jazzy）或 Docker。

### 方式一：本地仿真（单进程）

```bash
python3 simulation/main.py
```

- 每次运行自动生成随机障碍物、随机起点、随机终点。
- 会弹出 MuJoCo viewer，机器人沿 A\* 路径运动，到达终点后打印最终位置和误差。
- 需要图形界面支持（MuJoCo viewer）。

### 方式二：ROS2（仿真与导航两个节点）

在项目根目录构建并启动：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
export PYTHONPATH=<项目根目录>:$PYTHONPATH   # 让节点导入顶层共享模块
ros2 launch mujoco_navigation navigation.launch.py
```

另开终端验证话题：

```bash
source /opt/ros/jazzy/setup.bash
source <项目根目录>/ros2_ws/install/setup.bash
ros2 node list                       # 应看到 /mujoco_simulation_node 和 /navigation_node
ros2 topic echo /odom                # 仿真节点发布的机器人位置
ros2 topic echo /cmd_vel             # 导航节点发布的控制指令
```

机器人从 `(0, 0)` 导航到目标 `(-2, 2)`，到达后导航节点打印 `Goal reached. Publishing zero velocity.`

> 注意：`simulation_node` 依赖 mujoco，构建和启动请使用安装了 mujoco 的 Python 环境。

### 方式三：Docker（两个容器通过 ROS2 通信）

```bash
docker compose up -d          # 构建并启动仿真容器 + 导航容器
docker compose ps             # 查看容器状态
docker compose logs navigation  # 查看导航节点日志（可看到 Goal reached）
docker compose down           # 停止并移除容器
```

两个容器：

| 容器 | 镜像源 | 角色 |
|------|--------|------|
| `mujoco_simulation` | `simulation_docker/` | 运行 MuJoCo 仿真，发布 `/odom`、订阅 `/cmd_vel` |
| `navigation_stack` | `navigation_docker/` | 运行 A\* + PD 导航，订阅 `/odom`、发布 `/cmd_vel` |

## 核心模块说明

### 机器人模型（simulation/robot.xml）

- 圆柱体，三个关节：`tx`（X 滑动）、`ty`（Y 滑动）、`rz`（绕 Z 旋转）。
- 三个速度 actuator：`vx`、`vy`、`wz`，对应 X/Y 平移速度和角速度。
- 全向移动：可直接执行 X/Y 平移速度，同时旋转朝向。

### 地图（maps/terrain_map.pgm）

- 约定：`0` = 障碍物，`255` = 可通行。
- 障碍物按机器人半径 + 安全余量膨胀，使 A\* 可将机器人视为质点同时保留避障余量。
- `mapping/map_generator.py` 从固定场景 `simulation/world.xml` 生成固定地图；
  随机场景则通过 `simulation/random_environment.py` 自动生成。

### A* 路径规划（mapping/astar_planner.py）

- 四连通网格 A\*，欧氏距离启发式。
- `navigation/navigation_core.py` 负责地图读取、坐标转换、路径规划、waypoint 降采样和管理。

### PD 控制器（controller/pd_controller.py）

- 输入：当前位置 `(x, y)`、目标 waypoint `(x, y)`、控制周期 `dt`、当前 yaw。
- 输出：`vx / vy / wz`，带最大合速度限制与 yaw 朝向控制。

### 仿真核心（simulation/simulation_core.py）

- 加载场景、设置机器人初始位姿、获取位置/朝向、设置速度、推进仿真。
- 提供摄像头 RGB/深度图读取与 raycast 雷达式测距接口。

## 主要参数

在 `simulation/main.py` 中：

```python
environment = generate_random_environment(obstacle_count=16)   # 随机障碍物数量
navigation = NavigationCore(waypoint_step=4, kp=5.0, kd=0.15,
                            max_speed=4.0, reach_threshold=0.14)
```

在 `simulation/robot.xml` 中 `ctrlrange="-4.5 4.5"` 控制 X/Y 方向 actuator 速度范围。

## 传感器接口（实验性）

- `simulation_core.get_camera_image()`：机器人前向摄像头 RGB 图像。
- `simulation_core.get_camera_depth()`：深度图。
- `simulation_core.cast_navigation_rays()`：raycast 雷达式测距。
- `controller/camera_avoidance.py`、`depth_avoidance.py`：实验性避障控制器。

主流程保持稳定的 A\* + PD 路径跟踪，上述接口作为后续扩展方向保留。

## 任务完成情况

| # | 任务 | 状态 |
|---|------|------|
| 1 | GitHub 仓库 + 团队成员 | ✅ |
| 2 | MuJoCo 圆柱机器人（X/Y 速度 + 角速度控制） | ✅ |
| 3 | 场景中生成障碍物 | ✅ |
| 4 | 从 MuJoCo 获取地形地图 | ✅ |
| 5 | A\* 全局导航，随机选点并规划路径 | ✅ |
| 6 | PD 位置控制器跟踪路径（vx/vy/w） | ✅ |
| 7 | 加入 ROS2，仿真与控制器通过话题交换数据 | ✅ |
| 8 | Docker 容器（仿真与导航通过 ROS2 通信） | ✅ |
| 9 | 随机生成障碍物 | ✅ |
| 10 | 完善 README | ✅ |

## 常见问题

**Q: 机器人不动？**
A: 确认调用了 `simulation_core.set_velocity()` 并执行 `simulation_core.step()`；机器人模型需要有 `vx/vy/wz` 三个 actuator。

**Q: 找不到地图文件？**
A: 先运行 `python3 mapping/map_generator.py` 生成 `maps/terrain_map.pgm`，或直接运行 `simulation/main.py` 自动生成随机地图。

**Q: 卡死不动 / 一直绕圈？**
A: 当前雷达避障针对全向底盘采用侧向平移策略。若遇到异常，请先检查障碍物密度或调整 `obstacle_count`。

**Q: Docker 容器启动后看不到日志？**
A: 使用 `docker compose logs navigation` 查看；确认宿主机已安装 Docker 且守护进程正在运行。
