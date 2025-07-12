# 工业巡逻机器人系统

## 项目概述
本项目是一个基于ROS的工业巡逻机器人系统，集成了机器人控制、环境感知、目标检测和远程监控功能，适用于工业场景下的自动化巡逻任务。

## 主要功能模块
- **robot_control**: 包含机器人硬件驱动、定位导航和运动控制功能
- **server**: 提供前后端服务，实现远程监控和任务管理
- **yolo**: 基于YOLO的目标检测模块，支持人体识别和火灾检测

## 安装指南
### 环境要求
- Ubuntu 20.04/22.04
- ROS Noetic/Humble
- Python 3.8+
- CUDA 11.4+ (用于YOLO加速)

### 安装步骤
1. 克隆仓库并进入项目目录
```bash
git clone <仓库地址>
cd Embed
```

2. 安装ROS依赖
```bash
cd code/robot_control
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

3. 安装Python依赖
```bash
cd code/yolo
pip install -r requirements.txt
```

## 使用方法
### 启动机器人控制
```bash
cd code/robot_control
source install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

### 运行目标检测
```bash
cd code/yolo
python test.py
```

### 启动服务器
```bash
cd code/server/patrol-robot-backend
python app.py
# 同时启动前端
cd ../robot-frontend
npm start
```

## 项目结构
```
Embed/
├── code/
│   ├── robot_control/      # 机器人控制模块
│   ├── server/             # 服务端和前端
│   └── yolo/               # 目标检测模块
└── documentation/          # 项目文档
    ├── 初步方案设计.pdf
    ├── 工业巡逻机器人计划书.pdf
    ├── 技术文档.pdf
    └── 测试文档.pdf
```

## 文档资源
- [初步方案设计](documentation/初步方案设计.pdf)
- [工业巡逻机器人计划书](documentation/工业巡逻机器人计划书.pdf)
- [技术文档](documentation/技术文档.pdf)
- [测试文档](documentation/测试文档.pdf)
