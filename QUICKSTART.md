# 快速开始指南 / Quick Start Guide

## 1. 一键安装依赖 / One-Command Dependency Installation

```bash
# Ubuntu 22.04 with ROS 2 Humble
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-tf2-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    python3-pip

pip3 install numpy scipy
```

## 2. 克隆和编译 / Clone and Build

```bash
# 创建工作空间 / Create workspace
mkdir -p ~/ackermann_ws/src
cd ~/ackermann_ws/src

# 克隆项目 / Clone project
git clone https://github.com/zsanmu52/Pure-tracking-slam-automatic-navigation-system.git
cd ..

# 编译 / Build
source /opt/ros/humble/setup.bash
./src/Pure-tracking-slam-automatic-navigation-system/build.sh

# 如果build.sh不可执行，先添加执行权限 / If build.sh is not executable, add permission first
# chmod +x ./src/Pure-tracking-slam-automatic-navigation-system/build.sh

# 加载环境 / Source environment
source install/setup.bash
```

## 3. 运行阿克曼车辆（推荐）/ Run Ackermann Vehicle (Recommended)

### 终端 1：启动Gazebo仿真 / Terminal 1: Launch Gazebo Simulation
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch gazebo_modele ackermann_gazebo.launch.py
```

### 终端 2：启动导航系统 / Terminal 2: Launch Navigation System
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch nav_slam hybrid_astar_nav.launch.py
```

### 在RViz中设置目标 / Set Goal in RViz
1. 等待RViz启动 / Wait for RViz to launch
2. 点击工具栏的 "2D Goal Pose" / Click "2D Goal Pose" in toolbar
3. 在地图上点击并拖动设置目标位置和朝向 / Click and drag on map to set goal position and orientation
4. 观察车辆自动导航到目标 / Watch the vehicle navigate to the goal automatically

## 4. 运行差速驱动车辆（原版）/ Run Differential Drive Vehicle (Original)

### 终端 1：启动Gazebo仿真 / Terminal 1: Launch Gazebo Simulation
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch gazebo_modele gazebo.launch.py
```

### 终端 2：启动导航系统 / Terminal 2: Launch Navigation System
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch nav_slam 2dpoints.launch.py
```

## 5. 常用命令 / Useful Commands

### 查看话题列表 / List Topics
```bash
ros2 topic list
```

### 查看话题数据 / Echo Topic Data
```bash
# 查看速度命令 / View velocity commands
ros2 topic echo /cmd_vel

# 查看里程计 / View odometry
ros2 topic echo /odom

# 查看路径 / View path
ros2 topic echo /path
```

### 手动控制车辆 / Manual Vehicle Control
```bash
# 前进 / Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once

# 转向 / Turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}" --once

# 停止 / Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### 查看TF变换 / View TF Transforms
```bash
# 查看TF树 / View TF tree
ros2 run tf2_tools view_frames

# 查看特定变换 / View specific transform
ros2 run tf2_ros tf2_echo map base_link
```

## 6. 故障排除 / Troubleshooting

### 问题1：Gazebo无法启动 / Issue 1: Gazebo Won't Start
```bash
# 杀死残留进程 / Kill remaining processes
killall gzserver gzclient

# 清理Gazebo缓存 / Clear Gazebo cache
rm -rf ~/.gazebo/log/*
```

### 问题2：找不到包 / Issue 2: Package Not Found
```bash
# 重新编译 / Rebuild
cd ~/ackermann_ws
./src/Pure-tracking-slam-automatic-navigation-system/build.sh --clean
source install/setup.bash
```

### 问题3：Python导入错误 / Issue 3: Python Import Error
```bash
# 重新安装Python依赖 / Reinstall Python dependencies
pip3 install --upgrade numpy scipy
```

### 问题4：TF变换错误 / Issue 4: TF Transform Error
```bash
# 检查所有TF发布器是否运行 / Check if all TF publishers are running
ros2 node list

# 查看TF树 / View TF tree
ros2 run tf2_tools view_frames
# 生成的PDF文件在当前目录下 / Generated PDF is in current directory
```

## 7. 性能调优 / Performance Tuning

### 调整路径规划参数 / Adjust Path Planning Parameters
编辑配置文件 / Edit config file:
```bash
nano ~/ackermann_ws/src/Pure-tracking-slam-automatic-navigation-system/src/nav_slam/config/ackermann_params.yaml
```

关键参数 / Key parameters:
- `min_turning_radius`: 最小转弯半径（越小转弯越灵活）/ Minimum turning radius (smaller = more agile)
- `max_speed`: 最大速度 / Maximum speed
- `goal_tolerance`: 目标容差（越小越精确）/ Goal tolerance (smaller = more precise)
- `lookahead_distance`: 前视距离（影响路径跟踪平滑度）/ Lookahead distance (affects path tracking smoothness)

### 调整障碍物膨胀 / Adjust Obstacle Expansion
编辑文件 / Edit file:
```bash
nano ~/ackermann_ws/src/Pure-tracking-slam-automatic-navigation-system/src/nav_slam/nav_slam/hybrid_astar.py
```

修改第29行 / Modify line 29:
```python
expansion_size = 5  # 减小这个值使车辆可以通过更窄的空间 / Decrease for narrower passages
```

## 8. 更多帮助 / More Help

- 📖 完整文档：[README.md](README.md)
- 🎥 演示视频：https://www.bilibili.com/video/BV1kzEwzuEFw
- 📧 联系邮箱：clibang2022@163.com
- 🐛 问题报告：https://github.com/zsanmu52/Pure-tracking-slam-automatic-navigation-system/issues

---

## 项目结构快速参考 / Project Structure Quick Reference

```
Pure-tracking-slam-automatic-navigation-system/
├── build.sh                    # 一键编译脚本 / One-click build script
├── QUICKSTART.md              # 本文件 / This file
├── README.md                  # 完整文档 / Full documentation
└── src/
    ├── gazebo_modele/         # 仿真模型 / Simulation models
    │   ├── urdf/
    │   │   ├── ackermann_model.urdf    # 阿克曼车辆 / Ackermann vehicle
    │   │   └── model.urdf               # 差速车辆 / Diff drive vehicle
    │   └── launch/
    │       ├── ackermann_gazebo.launch.py   # 阿克曼启动 / Ackermann launch
    │       └── gazebo.launch.py             # 差速启动 / Diff drive launch
    │
    └── nav_slam/              # 导航算法 / Navigation algorithms
        ├── nav_slam/
        │   ├── hybrid_astar.py        # Hybrid A*算法 / Hybrid A* algorithm
        │   ├── astar.py               # A*算法 / A* algorithm
        │   └── start_nav.py           # 纯追踪控制 / Pure pursuit control
        ├── config/
        │   └── ackermann_params.yaml  # 参数配置 / Parameter config
        └── launch/
            ├── hybrid_astar_nav.launch.py   # Hybrid A*导航 / Hybrid A* nav
            └── 2dpoints.launch.py           # A*导航 / A* nav
```

---

🎉 **祝使用愉快！ / Enjoy!**
