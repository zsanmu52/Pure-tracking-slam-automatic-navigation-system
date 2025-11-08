#!/bin/bash
# 编译脚本 / Build Script
# 用于自动编译整个工作空间 / Automatically build the entire workspace

set -e  # 遇到错误立即退出 / Exit on error

# 颜色定义 / Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  ROS 2 Ackermann Navigation System${NC}"
echo -e "${GREEN}  编译脚本 / Build Script${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""

# 检查ROS 2环境 / Check ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}未检测到ROS 2环境，正在加载... / ROS 2 environment not detected, loading...${NC}"
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ ROS 2 Humble环境已加载 / ROS 2 Humble environment loaded${NC}"
    else
        echo -e "${RED}✗ 错误：未找到ROS 2 Humble / Error: ROS 2 Humble not found${NC}"
        echo -e "${YELLOW}请安装ROS 2 Humble或手动source环境 / Please install ROS 2 Humble or manually source the environment${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ 检测到ROS 2 $ROS_DISTRO环境 / ROS 2 $ROS_DISTRO environment detected${NC}"
fi

# 检查colcon / Check colcon
if ! command -v colcon &> /dev/null; then
    echo -e "${RED}✗ 错误：colcon未安装 / Error: colcon not installed${NC}"
    echo -e "${YELLOW}运行以下命令安装: / Run the following command to install:${NC}"
    echo "sudo apt install python3-colcon-common-extensions"
    exit 1
fi
echo -e "${GREEN}✓ colcon已安装 / colcon is installed${NC}"
echo ""

# 清理之前的编译（可选）/ Clean previous build (optional)
if [ "$1" == "--clean" ]; then
    echo -e "${YELLOW}清理之前的编译文件... / Cleaning previous build files...${NC}"
    rm -rf build install log
    echo -e "${GREEN}✓ 清理完成 / Cleanup complete${NC}"
    echo ""
fi

# 开始编译 / Start building
echo -e "${GREEN}开始编译... / Starting build...${NC}"
echo ""

# 编译所有包 / Build all packages
if colcon build --symlink-install; then
    echo ""
    echo -e "${GREEN}======================================${NC}"
    echo -e "${GREEN}✓ 编译成功！ / Build successful!${NC}"
    echo -e "${GREEN}======================================${NC}"
    echo ""
    echo -e "${YELLOW}下一步：/ Next steps:${NC}"
    echo "1. source install/setup.bash"
    echo "2. ros2 launch gazebo_modele ackermann_gazebo.launch.py  # 启动阿克曼仿真 / Launch Ackermann simulation"
    echo "3. ros2 launch nav_slam hybrid_astar_nav.launch.py       # 启动Hybrid A*导航 / Launch Hybrid A* navigation"
    echo ""
    echo -e "${YELLOW}或使用原版差速驱动: / Or use original differential drive:${NC}"
    echo "2. ros2 launch gazebo_modele gazebo.launch.py            # 启动差速仿真 / Launch diff drive simulation"
    echo "3. ros2 launch nav_slam 2dpoints.launch.py               # 启动A*导航 / Launch A* navigation"
else
    echo ""
    echo -e "${RED}======================================${NC}"
    echo -e "${RED}✗ 编译失败！ / Build failed!${NC}"
    echo -e "${RED}======================================${NC}"
    echo ""
    echo -e "${YELLOW}常见问题解决方案: / Common solutions:${NC}"
    echo "1. 确保所有依赖已安装 / Ensure all dependencies are installed"
    echo "2. 尝试清理编译: ./build.sh --clean / Try clean build: ./build.sh --clean"
    echo "3. 检查Python版本 (需要3.10+) / Check Python version (requires 3.10+)"
    exit 1
fi
