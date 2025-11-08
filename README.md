# Pure-tracking-slam-automatic-navigation-system
simulate，ros2，gazebo，navigation，slam



<!--
 * @作者: boxing
 * @b号: 喵了个水蓝蓝
 * @描述: README
-->
## 注意当前分支代码为humble版本

# 基于ROS2实现的差速机器人，slam（建图定位），路径规划（Hybrid A*），导航控制（纯追踪）

![image](https://github.com/user-attachments/assets/baac6889-d251-4891-8d21-c47fa4b45a33)


## 安装运行

```
git clone https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system.git
```

## 运行测试
###  启动仿真
```
ros2 launch gazebo_modele gazebo.launch.py
```
###  启动导航
```
ros2 launch nav_slam 2dpoints.launch.py
```
## 演示视频
https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f

## 特性更新

### Hybrid A* 路径规划算法
本系统现已采用 Hybrid A* 算法替代传统 A* 算法，具有以下优势：

- **考虑车辆运动学约束**：生成的路径符合差速机器人的转向和运动特性
- **方向感知**：规划过程中考虑机器人朝向，生成更自然的路径
- **更平滑的轨迹**：减少急转弯，提高导航的平稳性
- **智能回退机制**：当 Hybrid A* 无法找到路径时，自动回退到传统 A* 算法

#### 算法特点
- 状态空间：(x, y, θ) - 包含位置和方向
- 7种运动原语：直行、30°/45°/60° 左右转弯
- 角度离散化：72个方向（5°分辨率）
- 碰撞检测：沿路径插值检查，确保安全性
- 代价函数：综合考虑移动距离和转向代价

联系：clibang2022@163.com
