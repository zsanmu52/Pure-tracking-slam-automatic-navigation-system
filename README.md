# Pure-tracking-slam-automatic-navigation-system
simulate，ros2，gazebo，navigation，slam，Ackermann steering，Hybrid A*

<!--
 * @作者: boxing
 * @b号: 喵了个水蓝蓝
 * @描述: README
-->

## 注意当前分支代码为humble版本

# 基于ROS2实现的阿克曼转向机器人，SLAM（建图定位），路径规划（Hybrid A*），导航控制（纯追踪）

## 项目特性

### 车辆模型
- ✅ **阿克曼转向模型 (Ackermann Steering)** - 模拟真实汽车的转向机制
- ✅ **差速驱动模型 (Differential Drive)** - 原有的差速机器人支持

### 路径规划算法
- ✅ **Hybrid A* 算法** - 考虑车辆运动学约束的非完整路径规划
  - 支持Reeds-Shepp曲线路径生成
  - 适用于阿克曼转向车辆
- ✅ **A* 算法** - 传统的网格路径规划

### 导航控制
- ✅ **纯追踪控制 (Pure Pursuit)** - 平滑的路径跟踪控制器

### 仿真环境
- ✅ Gazebo 仿真环境
- ✅ RViz 可视化
- ✅ 3D世界模型

![image](https://github.com/user-attachments/assets/baac6889-d251-4891-8d21-c47fa4b45a33)

---

## 系统要求

- **操作系统**: Ubuntu 22.04 (推荐)
- **ROS版本**: ROS 2 Humble
- **Python**: Python 3.10+
- **依赖软件**:
  - Gazebo (Classic)
  - RViz2
  - colcon

---

## 安装步骤

### 1. 安装ROS 2 Humble

如果还未安装ROS 2 Humble，请按照[官方教程](https://docs.ros.org/en/humble/Installation.html)进行安装。

```bash
# 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS 2 apt源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. 安装依赖软件

```bash
# 安装Gazebo和相关插件
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

# 安装TF和机器人状态发布器
sudo apt install ros-humble-tf2-ros ros-humble-robot-state-publisher

# 安装joint state publisher
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui

# 安装RViz2
sudo apt install ros-humble-rviz2

# 安装colcon编译工具
sudo apt install python3-colcon-common-extensions

# 安装Python依赖
sudo apt install python3-pip
pip3 install numpy scipy
```

### 3. 克隆项目

```bash
# 创建工作空间
mkdir -p ~/ackermann_ws/src
cd ~/ackermann_ws/src

# 克隆项目
git clone https://github.com/zsanmu52/Pure-tracking-slam-automatic-navigation-system.git
cd ..
```

### 4. 编译项目

```bash
# 进入工作空间
cd ~/ackermann_ws

# 设置ROS 2环境
source /opt/ros/humble/setup.bash

# 编译
colcon build

# 如果遇到编译错误，可以尝试单独编译包
# colcon build --packages-select gazebo_modele
# colcon build --packages-select nav_slam
```

### 5. 设置环境变量

```bash
# 添加到~/.bashrc以便每次启动终端自动加载
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ackermann_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 或者手动source（仅当前终端有效）
source /opt/ros/humble/setup.bash
source ~/ackermann_ws/install/setup.bash
```

---

## 运行项目

### 方式一：阿克曼转向车辆 + Hybrid A* 路径规划（推荐）

#### 终端1：启动阿克曼车辆仿真
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch gazebo_modele ackermann_gazebo.launch.py
```

#### 终端2：启动Hybrid A*导航
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch nav_slam hybrid_astar_nav.launch.py
```

### 方式二：差速驱动车辆 + A* 路径规划（原版）

#### 终端1：启动差速车辆仿真
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch gazebo_modele gazebo.launch.py
```

#### 终端2：启动A*导航
```bash
cd ~/ackermann_ws
source install/setup.bash
ros2 launch nav_slam 2dpoints.launch.py
```

---

## 使用说明

### 1. 设置导航目标

在RViz2界面中：
1. 点击顶部工具栏的 **"2D Goal Pose"** 按钮
2. 在地图上点击并拖动鼠标设置目标位置和朝向
3. 系统将自动规划路径并开始导航

### 2. 查看规划路径

- **红色路径**: 原始规划路径（粗路径）
- **绿色路径**: 平滑后的路径（用于跟踪）

### 3. 控制车辆

导航启动后，车辆将自动跟踪规划的路径。你也可以手动控制：

```bash
# 手动控制（可选）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### 4. 查看话题

```bash
# 查看所有话题
ros2 topic list

# 查看里程计
ros2 topic echo /odom

# 查看路径
ros2 topic echo /path

# 查看点云数据
ros2 topic echo /points_raw
```

---

## 配置文件说明

### 车辆参数配置
配置文件位置：`src/nav_slam/config/ackermann_params.yaml`

主要参数：
- `wheel_base`: 轴距（米）
- `wheel_separation`: 轮距（米）
- `max_steering_angle`: 最大转向角（弧度）
- `min_turning_radius`: 最小转弯半径（米）

### URDF模型文件

- 阿克曼车辆: `src/gazebo_modele/urdf/ackermann_model.urdf`
- 差速驱动车辆: `src/gazebo_modele/urdf/model.urdf`

---

## 项目结构

```
Pure-tracking-slam-automatic-navigation-system/
├── src/
│   ├── gazebo_modele/              # Gazebo仿真模型包
│   │   ├── urdf/
│   │   │   ├── ackermann_model.urdf  # 阿克曼车辆模型
│   │   │   └── model.urdf            # 差速驱动车辆模型
│   │   ├── world/                    # Gazebo世界文件
│   │   │   ├── 2d.world
│   │   │   └── 3d.world
│   │   └── launch/
│   │       ├── ackermann_gazebo.launch.py  # 阿克曼车辆启动文件
│   │       └── gazebo.launch.py            # 差速车辆启动文件
│   │
│   └── nav_slam/                   # 导航和SLAM包
│       ├── nav_slam/
│       │   ├── hybrid_astar.py     # Hybrid A*算法实现
│       │   ├── astar.py            # A*算法实现
│       │   ├── start_nav.py        # 纯追踪控制器
│       │   ├── map_pub.py          # 地图发布节点
│       │   ├── odom_map_tf.py      # TF变换发布
│       │   └── points_pub_map.py   # 点云发布节点
│       ├── config/
│       │   ├── ackermann_params.yaml  # 阿克曼参数配置
│       │   └── rviz.rviz              # RViz配置文件
│       └── launch/
│           ├── hybrid_astar_nav.launch.py  # Hybrid A*导航启动
│           └── 2dpoints.launch.py          # A*导航启动
└── README.md
```

---

## 算法说明

### Hybrid A* 算法

Hybrid A*是A*算法的扩展，专门用于非完整约束的车辆路径规划：

**特点**：
- 考虑车辆运动学约束
- 使用连续空间表示状态（x, y, yaw）
- 支持Reeds-Shepp曲线连接到目标
- 生成可执行的平滑路径

**适用场景**：
- 阿克曼转向车辆
- 需要考虑转弯半径的场景
- 狭窄空间的路径规划

### 纯追踪控制器

纯追踪（Pure Pursuit）是一种几何路径跟踪算法：

**原理**：
1. 在路径上选择一个前视点
2. 计算车辆到前视点的转向角
3. 根据速度和转向角控制车辆

**优点**：
- 实现简单
- 鲁棒性好
- 适合平滑路径跟踪

---

## 常见问题

### 1. 编译错误：找不到rclpy

**解决方案**：
```bash
sudo apt install ros-humble-rclpy
```

### 2. Gazebo无法启动

**解决方案**：
```bash
# 检查Gazebo是否安装
gazebo --version

# 重新安装Gazebo
sudo apt install gazebo ros-humble-gazebo-ros-pkgs
```

### 3. 路径规划失败

**可能原因**：
- 障碍物膨胀过大
- 目标点在障碍物内
- 起点和终点距离太近

**解决方案**：
- 调整 `expansion_size` 参数
- 选择合适的目标点
- 增加 `goal_tolerance` 参数

### 4. 车辆不移动

**检查项**：
```bash
# 检查cmd_vel话题是否有数据
ros2 topic echo /cmd_vel

# 检查路径是否发布
ros2 topic echo /path

# 检查odom是否正常
ros2 topic echo /odom
```

### 5. TF变换错误

**解决方案**：
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查TF变换
ros2 run tf2_ros tf2_echo map base_link
```

---

## 演示视频

https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f

---

## 贡献指南

欢迎贡献代码！请遵循以下步骤：

1. Fork本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启Pull Request

---

## 许可证

本项目采用Apache License 2.0许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 联系方式

- **作者**: boxing / 喵了个水蓝蓝
- **Email**: clibang2022@163.com
- **B站**: [喵了个水蓝蓝](https://space.bilibili.com/)
- **GitHub**: [Ming2zun](https://github.com/Ming2zun)

---

## 致谢

感谢所有为这个项目做出贡献的开发者！

---

## 更新日志

### v2.0 (2025-01)
- ✨ 新增阿克曼转向车辆模型
- ✨ 实现Hybrid A*路径规划算法
- ✨ 支持Reeds-Shepp曲线路径生成
- 📝 完善README文档
- 🔧 添加配置文件支持

### v1.0 (2025-01)
- 🎉 初始版本发布
- ✨ 差速驱动机器人模型
- ✨ A*路径规划
- ✨ 纯追踪控制器
- ✨ SLAM建图定位
