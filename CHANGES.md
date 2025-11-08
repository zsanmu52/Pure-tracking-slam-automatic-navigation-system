# 变更日志 / Changelog

## v2.0.0 - 2025-01-08

### 🎉 重大更新 / Major Updates

本次更新实现了从差速驱动到阿克曼转向的完整升级，并引入了更先进的Hybrid A*路径规划算法。
This update implements a complete upgrade from differential drive to Ackermann steering and introduces the more advanced Hybrid A* path planning algorithm.

---

## 新增功能 / New Features

### 1. 阿克曼转向车辆模型 / Ackermann Steering Vehicle Model

**新文件 / New Files:**
- `src/gazebo_modele/urdf/ackermann_model.urdf`
- `src/gazebo_modele/launch/ackermann_gazebo.launch.py`

**特性 / Features:**
- ✨ 四轮独立建模，前轮可转向
- ✨ 使用 `libgazebo_ros_ackermann_drive.so` 插件
- ✨ 真实的车辆运动学模型
- ✨ 支持最小转弯半径约束
- ✨ 轴距 0.5m，轮距 0.4m
- ✨ 最大转向角 ±0.6 弧度 (约±34度)

**对比原有模型 / Comparison with Original:**
- 原模型：差速驱动，可原地转向
- 新模型：阿克曼转向，需要转弯半径

---

### 2. Hybrid A* 路径规划算法 / Hybrid A* Path Planning Algorithm

**新文件 / New Files:**
- `src/nav_slam/nav_slam/hybrid_astar.py`
- `src/nav_slam/launch/hybrid_astar_nav.launch.py`

**特性 / Features:**
- ✨ 考虑车辆运动学约束的路径规划
- ✨ 支持 Reeds-Shepp 曲线路径生成
- ✨ 5种运动原语（直行、左转、右转、大角度转向）
- ✨ 3D状态空间搜索 (x, y, yaw)
- ✨ 自动连接到目标点
- ✨ 生成平滑可执行路径

**算法实现细节 / Algorithm Details:**
```python
# 运动原语 / Motion Primitives
- (1.0, 0.0)   # 直行 / Straight
- (1.0, 0.3)   # 左转 / Left turn
- (1.0, -0.3)  # 右转 / Right turn
- (0.5, 0.5)   # 大角度左转 / Sharp left
- (0.5, -0.5)  # 大角度右转 / Sharp right
```

**Reeds-Shepp 路径类型 / Reeds-Shepp Path Types:**
- LSL, RSR (Left-Straight-Left, Right-Straight-Right)
- LSR, RSL (Left-Straight-Right, Right-Straight-Left)
- LRL, RLR (Left-Right-Left, Right-Left-Right)

**对比原有算法 / Comparison with Original A*:**
- 原算法：A*，网格搜索，不考虑朝向
- 新算法：Hybrid A*，连续空间，考虑车辆运动学

---

### 3. 配置文件 / Configuration Files

**新文件 / New Files:**
- `src/nav_slam/config/ackermann_params.yaml`

**可配置参数 / Configurable Parameters:**
```yaml
vehicle:
  wheel_base: 0.5              # 轴距
  wheel_separation: 0.4        # 轮距
  min_turning_radius: 1.5      # 最小转弯半径
  max_steering_angle: 0.6      # 最大转向角

hybrid_astar:
  grid_resolution: 0.2         # 网格分辨率
  angle_resolution: 15.0       # 角度分辨率
  max_iterations: 3000         # 最大迭代次数
  goal_tolerance: 0.5          # 目标容差

pure_pursuit:
  lookahead_distance: 0.5      # 前视距离
  min_speed: 0.6               # 最小速度
  max_speed: 1.5               # 最大速度
```

---

### 4. 构建和文档系统 / Build and Documentation System

**新文件 / New Files:**
- `build.sh` - 一键编译脚本
- `QUICKSTART.md` - 快速开始指南
- `README.md` - 完整文档（更新）
- `CHANGES.md` - 本文件
- `requirements.txt` - Python依赖
- `.gitignore` - Git忽略文件
- `src/nav_slam/CMakeLists.txt` - CMake构建文件
- `src/gazebo_modele/CMakeLists.txt` - CMake构建文件

**构建脚本特性 / Build Script Features:**
- ✅ 自动检测ROS 2环境
- ✅ 自动检测colcon
- ✅ 支持清理编译 (`--clean`)
- ✅ 中英文提示信息
- ✅ 编译状态实时反馈

**文档改进 / Documentation Improvements:**
- ✅ 完整的安装步骤
- ✅ 两种车辆模型的使用说明
- ✅ 详细的故障排除指南
- ✅ 参数调优建议
- ✅ 项目结构说明
- ✅ 算法原理解释

---

## 改进的功能 / Improved Features

### 1. 包依赖管理 / Package Dependency Management

**更新文件 / Updated Files:**
- `src/nav_slam/package.xml`
- `src/gazebo_modele/package.xml`
- `src/nav_slam/setup.py`

**改进 / Improvements:**
- ✅ 添加了所有必要的ROS 2依赖
- ✅ 更新了包描述和版本号
- ✅ 添加了Apache-2.0许可证声明
- ✅ 更新了维护者信息

---

## 兼容性 / Compatibility

### 向后兼容 / Backward Compatibility

✅ **完全保留原有功能**
- 原有的差速驱动模型（`model.urdf`）完全保留
- 原有的A*算法（`astar.py`）完全保留
- 原有的启动文件完全保留

### 使用方式 / Usage Options

**方式1：新功能（阿克曼 + Hybrid A*）**
```bash
# 终端1
ros2 launch gazebo_modele ackermann_gazebo.launch.py
# 终端2
ros2 launch nav_slam hybrid_astar_nav.launch.py
```

**方式2：原有功能（差速 + A*）**
```bash
# 终端1
ros2 launch gazebo_modele gazebo.launch.py
# 终端2
ros2 launch nav_slam 2dpoints.launch.py
```

---

## 技术细节 / Technical Details

### 车辆运动学模型 / Vehicle Kinematic Model

**阿克曼转向模型 / Ackermann Steering Model:**
```
δ_max = 0.6 rad
L = 0.5 m (wheelbase)
R_min = L / tan(δ_max) ≈ 1.5 m
```

**运动方程 / Motion Equations:**
```
x' = v * cos(θ)
y' = v * sin(θ)
θ' = (v / L) * tan(δ)
```

### 路径规划性能 / Path Planning Performance

**Hybrid A* vs A*:**
- 状态空间：3D (x, y, θ) vs 2D (x, y)
- 路径质量：考虑运动学约束 vs 仅几何最短
- 计算复杂度：O(n³) vs O(n²)
- 适用场景：非完整约束车辆 vs 全向移动

---

## 文件结构对比 / File Structure Comparison

### 新增文件 / New Files
```
+ src/gazebo_modele/urdf/ackermann_model.urdf
+ src/gazebo_modele/launch/ackermann_gazebo.launch.py
+ src/gazebo_modele/CMakeLists.txt
+ src/nav_slam/nav_slam/hybrid_astar.py
+ src/nav_slam/launch/hybrid_astar_nav.launch.py
+ src/nav_slam/config/ackermann_params.yaml
+ src/nav_slam/CMakeLists.txt
+ build.sh
+ QUICKSTART.md
+ CHANGES.md
+ requirements.txt
+ .gitignore
```

### 修改文件 / Modified Files
```
* README.md (完全重写)
* src/nav_slam/package.xml (添加依赖)
* src/gazebo_modele/package.xml (添加依赖)
* src/nav_slam/setup.py (添加hybrid_astar入口点)
```

### 保留文件 / Preserved Files
```
= src/gazebo_modele/urdf/model.urdf
= src/gazebo_modele/launch/gazebo.launch.py
= src/nav_slam/nav_slam/astar.py
= src/nav_slam/nav_slam/start_nav.py
= src/nav_slam/launch/2dpoints.launch.py
= (其他所有原有文件)
```

---

## 测试与验证 / Testing and Validation

### 已验证项 / Verified Items
- ✅ Python语法检查通过
- ✅ Launch文件语法正确
- ✅ URDF文件格式正确
- ✅ 包依赖关系完整
- ✅ 文档链接有效

### 需要用户测试 / User Testing Required
- ⏳ 实际编译测试（需要ROS 2环境）
- ⏳ Gazebo仿真运行
- ⏳ 路径规划功能
- ⏳ 导航控制性能

---

## 已知问题和限制 / Known Issues and Limitations

### 限制 / Limitations
1. Hybrid A*算法计算复杂度较高，大地图可能较慢
2. 阿克曼车辆无法原地转向，需要更大空间
3. Reeds-Shepp曲线在某些情况下可能找不到路径

### 建议 / Recommendations
1. 在大地图中使用时，可以减小搜索范围
2. 设置目标点时考虑车辆转弯半径
3. 调整 `goal_tolerance` 参数以适应不同场景

---

## 未来计划 / Future Plans

### 可能的改进 / Potential Improvements
- [ ] 添加动态障碍物避障
- [ ] 实现路径重规划
- [ ] 优化Hybrid A*性能
- [ ] 添加更多车辆模型
- [ ] 集成更多传感器
- [ ] 添加单元测试

---

## 参考资料 / References

### 论文 / Papers
- Hybrid A*: "Practical Search Techniques in Path Planning for Autonomous Driving"
- Reeds-Shepp Curves: "Optimal paths for a car that goes both forwards and backwards"

### 相关链接 / Related Links
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Gazebo: http://gazebosim.org/
- 项目仓库 / Repository: https://github.com/zsanmu52/Pure-tracking-slam-automatic-navigation-system

---

## 致谢 / Acknowledgments

感谢以下项目和资源的启发：
Thanks to the following projects and resources for inspiration:

- ROS 2 Navigation Stack
- Open Motion Planning Library (OMPL)
- PythonRobotics
- The Robotics community

---

## 联系方式 / Contact

- **作者 / Author**: boxing / 喵了个水蓝蓝
- **Email**: clibang2022@163.com
- **B站 / Bilibili**: [喵了个水蓝蓝](https://space.bilibili.com/)
- **GitHub**: [Ming2zun](https://github.com/Ming2zun)

---

**更新时间 / Last Updated**: 2025-01-08
**版本 / Version**: 2.0.0
