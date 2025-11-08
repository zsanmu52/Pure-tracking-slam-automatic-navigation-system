#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
#                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import math
from scipy.interpolate import BSpline
import time


expansion_size = 5  # 障碍物膨胀大小


# 处理成本图数据，扩展障碍物
def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)
    wall_mask = data == 100
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            shifted_mask = np.roll(wall_mask, (i, j), axis=(0, 1))
            data[shifted_mask] = 100
    data = data * resolution
    return data


def bezier_smoothing(array, num_points):
    """路径平滑处理"""
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        
        dx = np.diff(x, prepend=x[0])
        dy = np.diff(y, prepend=y[0])
        chord_lengths = np.sqrt(dx**2 + dy**2)
        t = np.concatenate(([0], np.cumsum(chord_lengths)))
        t /= t[-1]
        
        k = min(3, num_points - 1)
        
        t_knots = np.concatenate(([0] * k, t, [1] * k))
        x_padded = np.pad(x, (k, k), 'edge')
        y_padded = np.pad(y, (k, k), 'edge')
        
        spline_x = BSpline(t_knots, x_padded, k, extrapolate=False)
        spline_y = BSpline(t_knots, y_padded, k, extrapolate=False)
        
        t_new = np.linspace(0, 1, num_points)
        x_smoothed = spline_x(t_new)
        y_smoothed = spline_y(t_new)
        
        path = np.column_stack((x_smoothed, y_smoothed))
    except Exception as e:
        path = array
    return path


class ReedsSheppPath:
    """Reeds-Shepp曲线路径生成器"""
    
    def __init__(self, turning_radius=1.0):
        self.turning_radius = turning_radius
    
    def calc_paths(self, start_x, start_y, start_yaw, end_x, end_y, end_yaw, step_size=0.1):
        """计算Reeds-Shepp路径"""
        dx = end_x - start_x
        dy = end_y - start_y
        dth = end_yaw - start_yaw
        
        # 转换到局部坐标系
        c = math.cos(start_yaw)
        s = math.sin(start_yaw)
        x = c * dx + s * dy
        y = -s * dx + c * dy
        
        # 简化的Reeds-Shepp路径：使用圆弧+直线+圆弧
        paths = []
        
        # CSC路径 (Curve-Straight-Curve)
        paths.extend(self._calc_CSC(x, y, dth))
        
        # CCC路径 (Curve-Curve-Curve)
        paths.extend(self._calc_CCC(x, y, dth))
        
        if not paths:
            # 如果没有找到路径，返回简单直线路径
            return self._simple_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, step_size)
        
        # 选择最短路径
        best_path = min(paths, key=lambda p: p['length'])
        
        # 生成路径点
        return self._generate_path(start_x, start_y, start_yaw, best_path, step_size)
    
    def _calc_CSC(self, x, y, phi):
        """计算CSC路径"""
        paths = []
        
        # LSL路径
        t, u, v = self._LSL(x, y, phi)
        if t is not None and u is not None and v is not None:
            paths.append({'type': 'LSL', 't': t, 'u': u, 'v': v, 
                         'length': abs(t) + abs(u) + abs(v)})
        
        # RSR路径
        t, u, v = self._RSR(x, y, phi)
        if t is not None and u is not None and v is not None:
            paths.append({'type': 'RSR', 't': t, 'u': u, 'v': v,
                         'length': abs(t) + abs(u) + abs(v)})
        
        # LSR路径
        t, u, v = self._LSR(x, y, phi)
        if t is not None and u is not None and v is not None:
            paths.append({'type': 'LSR', 't': t, 'u': u, 'v': v,
                         'length': abs(t) + abs(u) + abs(v)})
        
        # RSL路径
        t, u, v = self._RSL(x, y, phi)
        if t is not None and u is not None and v is not None:
            paths.append({'type': 'RSL', 't': t, 'u': u, 'v': v,
                         'length': abs(t) + abs(u) + abs(v)})
        
        return paths
    
    def _calc_CCC(self, x, y, phi):
        """计算CCC路径"""
        paths = []
        
        # LRL路径
        t, u, v = self._LRL(x, y, phi)
        if t is not None and u is not None and v is not None:
            paths.append({'type': 'LRL', 't': t, 'u': u, 'v': v,
                         'length': abs(t) + abs(u) + abs(v)})
        
        # RLR路径
        t, u, v = self._RLR(x, y, phi)
        if t is not None and u is not None and v is not None:
            paths.append({'type': 'RLR', 't': t, 'u': u, 'v': v,
                         'length': abs(t) + abs(u) + abs(v)})
        
        return paths
    
    def _LSL(self, x, y, phi):
        """Left-Straight-Left路径"""
        u, t = self._R(x - math.sin(phi), y - 1.0 + math.cos(phi))
        if t >= 0.0 and u >= 0.0:
            v = self._M(phi - t)
            if v >= 0.0:
                return t, u, v
        return None, None, None
    
    def _RSR(self, x, y, phi):
        """Right-Straight-Right路径"""
        u, t = self._R(x + math.sin(phi), y - 1.0 - math.cos(phi))
        if t >= 0.0 and u >= 0.0:
            v = self._M(phi - t)
            if v >= 0.0:
                return -t, -u, -v
        return None, None, None
    
    def _LSR(self, x, y, phi):
        """Left-Straight-Right路径"""
        u1, t1 = self._R(x + math.sin(phi), y - 1.0 - math.cos(phi))
        u1 = u1 ** 2
        if u1 >= 4.0:
            u = math.sqrt(u1 - 4.0)
            theta = math.atan2(2.0, u)
            t = self._M(t1 + theta)
            v = self._M(t - phi)
            if t >= 0.0 and v >= 0.0:
                return t, u, v
        return None, None, None
    
    def _RSL(self, x, y, phi):
        """Right-Straight-Left路径"""
        u1, t1 = self._R(x - math.sin(phi), y - 1.0 + math.cos(phi))
        u1 = u1 ** 2
        if u1 >= 4.0:
            u = math.sqrt(u1 - 4.0)
            theta = math.atan2(2.0, u)
            t = self._M(t1 + theta)
            v = self._M(t - phi)
            if t >= 0.0 and v >= 0.0:
                return -t, -u, -v
        return None, None, None
    
    def _LRL(self, x, y, phi):
        """Left-Right-Left路径"""
        u1, t1 = self._R(x - math.sin(phi), y - 1.0 + math.cos(phi))
        if u1 <= 4.0:
            u = -2.0 * math.asin(0.25 * u1)
            t = self._M(t1 + 0.5 * u + math.pi)
            v = self._M(phi - t + u)
            if t >= 0.0 and u <= 0.0:
                return t, u, v
        return None, None, None
    
    def _RLR(self, x, y, phi):
        """Right-Left-Right路径"""
        u1, t1 = self._R(x + math.sin(phi), y - 1.0 - math.cos(phi))
        if u1 <= 4.0:
            u = 2.0 * math.asin(0.25 * u1)
            t = self._M(t1 - 0.5 * u + math.pi)
            v = self._M(phi - t - u)
            if t >= 0.0 and u >= 0.0:
                return -t, -u, -v
        return None, None, None
    
    def _R(self, x, y):
        """计算极坐标"""
        r = math.sqrt(x ** 2 + y ** 2)
        theta = math.atan2(y, x)
        return r, theta
    
    def _M(self, theta):
        """角度归一化到[0, 2*pi)"""
        while theta < 0.0:
            theta += 2.0 * math.pi
        while theta >= 2.0 * math.pi:
            theta -= 2.0 * math.pi
        return theta
    
    def _simple_path(self, start_x, start_y, start_yaw, end_x, end_y, end_yaw, step_size):
        """简单直线路径"""
        distance = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)
        num_points = max(int(distance / step_size), 2)
        x_list = np.linspace(start_x, end_x, num_points)
        y_list = np.linspace(start_y, end_y, num_points)
        yaw_list = [math.atan2(end_y - start_y, end_x - start_x)] * num_points
        return x_list, y_list, yaw_list
    
    def _generate_path(self, start_x, start_y, start_yaw, path_data, step_size):
        """根据路径参数生成实际路径点"""
        x_list = [start_x]
        y_list = [start_y]
        yaw_list = [start_yaw]
        
        x, y, yaw = start_x, start_y, start_yaw
        
        # 根据路径类型生成点
        path_type = path_data['type']
        t, u, v = path_data['t'], path_data['u'], path_data['v']
        
        # 第一段
        x, y, yaw = self._interpolate_segment(x, y, yaw, t, path_type[0], step_size, x_list, y_list, yaw_list)
        
        # 第二段
        x, y, yaw = self._interpolate_segment(x, y, yaw, u, path_type[1], step_size, x_list, y_list, yaw_list)
        
        # 第三段
        x, y, yaw = self._interpolate_segment(x, y, yaw, v, path_type[2], step_size, x_list, y_list, yaw_list)
        
        return np.array(x_list), np.array(y_list), np.array(yaw_list)
    
    def _interpolate_segment(self, x, y, yaw, length, seg_type, step_size, x_list, y_list, yaw_list):
        """插值单个路径段"""
        num_steps = max(int(abs(length) / step_size), 1)
        
        for i in range(1, num_steps + 1):
            step = (length / num_steps) * i
            
            if seg_type == 'S':  # Straight
                x_new = x + step * math.cos(yaw)
                y_new = y + step * math.sin(yaw)
                yaw_new = yaw
            elif seg_type == 'L':  # Left turn
                x_new = x + self.turning_radius * (math.sin(yaw + step) - math.sin(yaw))
                y_new = y - self.turning_radius * (math.cos(yaw + step) - math.cos(yaw))
                yaw_new = yaw + step
            elif seg_type == 'R':  # Right turn
                x_new = x - self.turning_radius * (math.sin(yaw - step) - math.sin(yaw))
                y_new = y + self.turning_radius * (math.cos(yaw - step) - math.cos(yaw))
                yaw_new = yaw - step
            else:
                continue
            
            x_list.append(x_new)
            y_list.append(y_new)
            yaw_list.append(yaw_new)
        
        return x_list[-1], y_list[-1], yaw_list[-1]


class Node3D:
    """3D节点（x, y, yaw）用于Hybrid A*"""
    
    def __init__(self, x, y, yaw, cost, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.cost < other.cost


def hybrid_astar(start, goal, grid, resolution, origin_x, origin_y):
    """
    Hybrid A*算法实现
    start: (x, y, yaw) 起始位置和朝向
    goal: (x, y, yaw) 目标位置和朝向
    grid: 成本图
    resolution: 地图分辨率
    origin_x, origin_y: 地图原点
    """
    
    # 车辆参数
    wheel_base = 0.5  # 轴距
    turning_radius = 1.5  # 最小转弯半径
    
    # 运动原语（前进，左转，右转）
    motion_primitives = [
        (1.0, 0.0),      # 直行
        (1.0, 0.3),      # 左转
        (1.0, -0.3),     # 右转
        (0.5, 0.5),      # 大角度左转
        (0.5, -0.5),     # 大角度右转
    ]
    
    # 初始化
    start_node = Node3D(start[0], start[1], start[2], 0.0)
    goal_pos = (goal[0], goal[1])
    
    open_set = []
    heapq.heappush(open_set, (0.0, 0, start_node))
    closed_set = {}
    
    # 用于生成唯一ID
    node_count = 1
    
    # 角度分辨率
    yaw_resolution = np.deg2rad(15)  # 15度
    
    def heuristic(node):
        """启发式函数：欧几里得距离"""
        dx = goal_pos[0] - node.x
        dy = goal_pos[1] - node.y
        return math.sqrt(dx * dx + dy * dy)
    
    def get_grid_index(x, y):
        """获取网格索引"""
        col = int((x - origin_x) / resolution)
        row = int((y - origin_y) / resolution)
        return row, col
    
    def is_valid(x, y):
        """检查位置是否有效"""
        row, col = get_grid_index(x, y)
        rows, cols = grid.shape
        if 0 <= row < rows and 0 <= col < cols:
            return grid[row, col] != 100
        return False
    
    def get_node_key(x, y, yaw):
        """生成节点的唯一键"""
        grid_x = int(x / resolution)
        grid_y = int(y / resolution)
        grid_yaw = int(yaw / yaw_resolution)
        return (grid_x, grid_y, grid_yaw)
    
    # Reeds-Shepp路径规划器
    rs = ReedsSheppPath(turning_radius)
    
    # 搜索
    max_iterations = 3000
    iteration = 0
    
    while open_set and iteration < max_iterations:
        iteration += 1
        
        _, _, current = heapq.heappop(open_set)
        
        # 检查是否到达目标
        dist_to_goal = math.sqrt((current.x - goal_pos[0]) ** 2 + (current.y - goal_pos[1]) ** 2)
        if dist_to_goal < 0.5:  # 0.5米以内认为到达目标
            # 使用Reeds-Shepp曲线连接到目标
            try:
                rs_x, rs_y, rs_yaw = rs.calc_paths(
                    current.x, current.y, current.yaw,
                    goal[0], goal[1], goal[2], step_size=0.2
                )
                
                # 检查RS路径是否有效
                valid_rs = True
                for rx, ry in zip(rs_x, rs_y):
                    if not is_valid(rx, ry):
                        valid_rs = False
                        break
                
                if valid_rs:
                    # 构建完整路径
                    path = []
                    node = current
                    while node:
                        path.append((node.x, node.y, node.yaw))
                        node = node.parent
                    path.reverse()
                    
                    # 添加RS路径
                    for rx, ry, ryaw in zip(rs_x, rs_y, rs_yaw):
                        path.append((rx, ry, ryaw))
                    
                    return path
            except:
                pass
        
        # 标记为已访问
        node_key = get_node_key(current.x, current.y, current.yaw)
        if node_key in closed_set:
            continue
        closed_set[node_key] = True
        
        # 扩展节点
        for v, delta in motion_primitives:
            # 使用自行车运动模型
            dt = 0.3  # 时间步长
            
            new_yaw = current.yaw + (v / wheel_base) * math.tan(delta) * dt
            new_x = current.x + v * math.cos(current.yaw) * dt
            new_y = current.y + v * math.sin(current.yaw) * dt
            
            # 检查新位置是否有效
            if not is_valid(new_x, new_y):
                continue
            
            # 计算成本
            step_cost = math.sqrt((new_x - current.x) ** 2 + (new_y - current.y) ** 2)
            # 增加转向惩罚
            step_cost += abs(delta) * 0.5
            
            new_cost = current.cost + step_cost
            new_node = Node3D(new_x, new_y, new_yaw, new_cost, current)
            
            new_key = get_node_key(new_x, new_y, new_yaw)
            if new_key not in closed_set:
                priority = new_cost + heuristic(new_node)
                heapq.heappush(open_set, (priority, node_count, new_node))
                node_count += 1
    
    # 未找到路径，返回空
    return []


# 导航控制节点类
class NavigationControl(Node):
    def __init__(self):
        super().__init__('HybridAStarNavigation')
        
        # 订阅器和发布器
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'combined_grid', self.map_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        self.path_publisher2 = self.create_publisher(Path, 'path2', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.pose_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # 状态变量
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.goal = None
        self.path = None
        self.path2 = None
        
        # 定时发布路径
        self.create_timer(0.1, self.publish_path)
    
    def goal_callback(self, msg):
        """目标位置回调"""
        # 从四元数提取偏航角
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        goal_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.goal = (msg.pose.position.x, msg.pose.position.y, goal_yaw)
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # 从四元数提取偏航角
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def map_callback(self, msg):
        """地图数据回调"""
        if self.goal is None:
            return
        
        distance = math.hypot(self.x - self.goal[0], self.y - self.goal[1])
        
        if distance > 0.3:  # 距离目标较远时规划路径
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            
            # 处理成本图
            data = costmap(msg.data, msg.info.width, msg.info.height, resolution)
            
            # 将-1到5之间的值设为1，其余值设为100
            data[(data >= -2) & (data <= 5)] = 1
            data[(data < -2) | (data > 5)] = 100
            
            # 起点和终点（包含朝向）
            start = (self.x, self.y, self.yaw)
            goal = self.goal
            
            # 使用Hybrid A*规划路径
            path = hybrid_astar(start, goal, data, resolution, origin_x, origin_y)
            
            if len(path) > 5:
                # 提取x, y坐标
                paths = [(p[0], p[1]) for p in path]
                self.path = paths
                
                # 平滑路径
                try:
                    self.path2 = bezier_smoothing(np.array(paths), len(paths))
                except:
                    self.path2 = paths
    
    def publish_path(self):
        """发布路径"""
        if self.path is None or len(self.path) == 0:
            return
        
        # 发布原始路径
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for (x, y) in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_msg.poses.append(pose)
        
        self.path_publisher.publish(path_msg)
        
        # 发布平滑路径
        if self.path2 is not None:
            path2_msg = Path()
            path2_msg.header.frame_id = 'map'
            path2_msg.header.stamp = self.get_clock().now().to_msg()
            
            for (x, y) in self.path2:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                path2_msg.poses.append(pose)
            
            self.path_publisher2.publish(path2_msg)


def main(args=None):
    rclpy.init(args=args)
    navigation_control = NavigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
