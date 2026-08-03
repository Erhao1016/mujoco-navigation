# mujoco-navigation
ROS2 Navigation with MuJoCo

task1 done

task2 done

task3 done

task4 done

task5 done

Installation required
pip install mujoco numpy pillow pyyaml matplotlib scipy

# 项目结构
mujoco-navigation/
├── controller/
│   └── controller.py          # 机器人速度控制器
├── mapping/
│   ├── map_generator.py       # 从 MuJoCo 场景生成地图 + A* 路径规划
│   └── planner/
│       └── astar_planner.py   # A* 算法核心实现
├── simulation/
│   ├── main.py                # 主仿真入口
│   ├── robot.xml              # 机器人模型定义
│   └── world.xml              # 场景定义（地面 + 障碍物）
├── simulation_docker/
│   └── Dockerfile             # 仿真容器配置
├── navigation_docker/
│   └── Dockerfile             # 导航容器配置
├── docker-compose.yml         # 多容器编排
└── README.md                  # 本文件
