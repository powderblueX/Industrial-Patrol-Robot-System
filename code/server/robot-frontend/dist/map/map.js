"use strict";

// map.js
const app = getApp();
Page({
  data: {
    mapScale: 1,
    // 地图缩放比例
    robotPosition: {
      x: 150,
      y: 150
    },
    lastTouchPoint: null,
    // 上次触摸点，用于地图拖动
    alarmList: [] // 告警列表
  },
  onLoad() {
    // 获取屏幕宽高以调整位置
    const systemInfo = wx.getSystemInfoSync();
    this.screenWidth = systemInfo.windowWidth;
    this.screenHeight = systemInfo.windowHeight;

    // 设置初始机器人位置
    this.setData({
      robotPosition: {
        x: this.screenWidth / 2,
        y: this.screenHeight / 2
      }
    });

    // 设置目标位置和移动参数
    this.targetPosition = {
      x: this.screenWidth / 2,
      y: this.screenHeight / 2
    };
    this.movementSpeed = 2; // 每次移动的最大像素数
    this.pathPoints = this.generatePathPoints(); // 生成路径点
    this.currentPathIndex = 0;

    // 启动机器人平滑移动
    this.startSmoothMovement();

    // 模拟告警检测
    this.startAlarmDetection();
  },
  // 生成巡检路径点
  generatePathPoints() {
    // 在屏幕范围内生成一系列路径点
    const centerX = this.screenWidth / 2;
    const centerY = this.screenHeight / 2;
    const radius = Math.min(this.screenWidth, this.screenHeight) * 0.3;
    const points = [];
    // 添加一些固定路径点
    points.push({
      x: centerX,
      y: centerY
    });
    points.push({
      x: centerX + radius * 0.8,
      y: centerY - radius * 0.5
    });
    points.push({
      x: centerX + radius,
      y: centerY
    });
    points.push({
      x: centerX + radius * 0.5,
      y: centerY + radius * 0.8
    });
    points.push({
      x: centerX - radius * 0.3,
      y: centerY + radius
    });
    points.push({
      x: centerX - radius,
      y: centerY + radius * 0.2
    });
    points.push({
      x: centerX - radius * 0.8,
      y: centerY - radius * 0.6
    });
    points.push({
      x: centerX,
      y: centerY - radius
    });
    points.push({
      x: centerX,
      y: centerY
    });
    return points;
  },
  // 启动机器人平滑移动
  startSmoothMovement() {
    // 每秒刷新多次位置，实现平滑移动
    this.movementTimer = setInterval(() => {
      // 获取当前位置
      const currentPosition = this.data.robotPosition;

      // 移动到下一个目标点
      if (this.isNearTarget(currentPosition, this.targetPosition)) {
        // 如果接近目标点，更换目标点
        this.currentPathIndex = (this.currentPathIndex + 1) % this.pathPoints.length;
        this.targetPosition = this.pathPoints[this.currentPathIndex];
      }

      // 计算朝目标点移动的新位置
      const newPosition = this.moveTowardsTarget(currentPosition, this.targetPosition);

      // 更新机器人位置
      this.setData({
        robotPosition: newPosition
      });
    }, 100); // 每100毫秒更新一次位置，实现更平滑的移动
  },
  // 检查是否接近目标点
  isNearTarget(current, target) {
    const distance = Math.sqrt(Math.pow(current.x - target.x, 2) + Math.pow(current.y - target.y, 2));
    return distance < 5; // 如果距离小于5像素，认为已到达
  },
  // 向目标点移动一小步
  moveTowardsTarget(current, target) {
    // 计算方向
    const dx = target.x - current.x;
    const dy = target.y - current.y;

    // 计算距离
    const distance = Math.sqrt(dx * dx + dy * dy);
    if (distance < 1) {
      return target; // 如果已经非常接近，直接返回目标点
    }

    // 计算单位方向向量
    const ux = dx / distance;
    const uy = dy / distance;

    // 计算实际移动距离（限制最大速度）
    const moveDistance = Math.min(this.movementSpeed, distance);

    // 计算新位置
    return {
      x: current.x + ux * moveDistance,
      y: current.y + uy * moveDistance
    };
  },
  // 启动告警检测
  startAlarmDetection() {
    this.alarmTimer = setInterval(() => {
      // 10%概率生成告警
      if (Math.random() < 0.1) {
        this.addNewAlarm();
      }
    }, 1000); // 每秒检测一次
  },
  // 添加新告警，位置与机器人当前位置一致
  addNewAlarm() {
    const alarmTypes = ['温度异常', '湿度异常', '烟雾报警', '通信异常', '电源异常', '设备故障'];
    const levels = ['high', 'medium', 'low'];
    const randomType = alarmTypes[Math.floor(Math.random() * alarmTypes.length)];
    const randomLevel = levels[Math.floor(Math.random() * levels.length)];

    // 使用机器人当前位置作为告警位置
    const x = this.data.robotPosition.x;
    const y = this.data.robotPosition.y;

    // 生成告警ID (时间戳+随机数)
    const alarmId = 'alarm_' + Date.now() + '_' + Math.floor(Math.random() * 1000);

    // 创建新告警
    const newAlarm = {
      id: alarmId,
      type: randomType,
      level: randomLevel,
      position: {
        x,
        y
      },
      time: this.formatTime(new Date())
    };

    // 添加到告警列表
    const alarmList = this.data.alarmList.concat([newAlarm]);
    this.setData({
      alarmList
    });

    // 3秒后移除告警
    setTimeout(() => {
      this.removeAlarm(alarmId);
    }, 3000);
  },
  // 移除指定ID的告警
  removeAlarm(alarmId) {
    const alarmList = this.data.alarmList.filter(item => item.id !== alarmId);
    this.setData({
      alarmList
    });
  },
  // 格式化时间
  formatTime(date) {
    const hour = date.getHours().toString().padStart(2, '0');
    const minute = date.getMinutes().toString().padStart(2, '0');
    const second = date.getSeconds().toString().padStart(2, '0');
    return `${hour}:${minute}:${second}`;
  },
  // 地图缩放：放大
  zoomIn() {
    let scale = this.data.mapScale;
    scale = Math.min(scale + 0.2, 3.0); // 限制最大缩放倍数为3倍
    this.setData({
      mapScale: scale
    });
  },
  // 地图缩放：缩小
  zoomOut() {
    let scale = this.data.mapScale;
    scale = Math.max(scale - 0.2, 0.5); // 限制最小缩放倍数为0.5倍
    this.setData({
      mapScale: scale
    });
  },
  // 地图缩放：重置
  resetMap() {
    this.setData({
      mapScale: 1.0
    });
  },
  // 触摸开始事件
  touchStart(e) {
    if (e.touches.length === 1) {
      this.setData({
        lastTouchPoint: {
          x: e.touches[0].pageX,
          y: e.touches[0].pageY
        }
      });
    }
  },
  // 触摸移动事件 - 用于地图拖动
  touchMove(e) {
    // 这里仅模拟拖动效果，实际实现需要改变地图容器的位置或使用transform
    if (e.touches.length === 1 && this.data.lastTouchPoint) {
      // 计算移动距离
      const deltaX = e.touches[0].pageX - this.data.lastTouchPoint.x;
      const deltaY = e.touches[0].pageY - this.data.lastTouchPoint.y;

      // 更新最后触摸点
      this.setData({
        lastTouchPoint: {
          x: e.touches[0].pageX,
          y: e.touches[0].pageY
        }
      });

      // 实际应用中，这里需要更新地图位置
      console.log(`地图移动: ${deltaX}, ${deltaY}`);
    }
  },
  // 触摸结束事件
  touchEnd() {
    this.setData({
      lastTouchPoint: null
    });
  },
  onUnload() {
    // 清除所有定时器
    if (this.movementTimer) {
      clearInterval(this.movementTimer);
    }
    if (this.alarmTimer) {
      clearInterval(this.alarmTimer);
    }
  }
});