// map.js
const app = getApp()

// WebSocket配置
const WS_URL = 'ws://100.81.120.217:8080/ws/robot'  // 修改为实际IP地址

// 重连配置
const RECONNECT_CONFIG = {
  maxRetries: 5,
  currentRetry: 0,
  retryDelay: 3000,
  maxDelay: 10000
}

// 消息主题
const TOPICS = {
  POSITION: '/topic/robot/position',
  ALERT: '/topic/robot/alert'
}

// 告警类型配置
const ALERT_TYPES = {
  INTRUSION: {
    color: '#ff0000',  // 入侵告警红色
    name: '入侵'
  },
  FIRE: {
    color: '#ffa500',  // 火灾告警橙色
    name: '火灾'
  },
  SMOKE: {
    color: '#ffff00',  // 烟雾告警黄色
    name: '烟雾'
  },
  // 可以添加更多告警类型
}

// 视角跟随配置
const FOLLOW_CONFIG = {
  enabled: true,           // 是否启用跟随模式
  followScale: 2.0,        // 跟随模式下的缩放倍数
  smoothTransition: true,  // 是否使用平滑过渡
  transitionDuration: 300, // 过渡动画时长(ms)
  centerOffset: {          // 机器人在屏幕中的偏移位置
    x: 0,                  // 0表示屏幕中心，-0.2表示偏左20%
    y: -0.1                // -0.1表示偏上10%，给下方留更多空间显示信息
  }
}

Page({
  data: {
    mapScale: 1, // 地图缩放比例
    mapTransform: {
      translateX: 0,
      translateY: 0,
      scale: 1
    },
    robotPosition: {
      x: 150,
      y: 150
    },
    lastTouchPoint: null, // 上次触摸点，用于地图拖动
    alarmList: [], // 告警列表
    alerts: [], // 当前显示的告警标记列表
    connected: false, // WebSocket连接状态
    isManualClosed: false, // 标记是否手动关闭连接
    ALERT_TYPES: ALERT_TYPES, // 添加告警类型配置到data中
    // 视角跟随相关数据
    followMode: FOLLOW_CONFIG.enabled,
    screenCenter: { x: 0, y: 0 }
  },

  onLoad() {
    // 获取屏幕宽高以调整位置
    const systemInfo = wx.getSystemInfoSync();
    this.screenWidth = systemInfo.windowWidth;
    this.screenHeight = systemInfo.windowHeight;
    console.log('屏幕宽度:', this.screenWidth, '屏幕高度:', this.screenHeight);

    // 计算屏幕中心点
    const screenCenter = {
      x: this.screenWidth / 2,
      y: this.screenHeight / 2
    };

    // 设置初始机器人位置和屏幕中心
    this.setData({
      robotPosition: {
        x: this.screenWidth / 2,
        y: this.screenHeight / 2
      },
      screenCenter: screenCenter,
      mapTransform: {
        translateX: 0,
        translateY: 0,
        scale: FOLLOW_CONFIG.enabled ? FOLLOW_CONFIG.followScale : 1
      }
    });

    // 如果启用跟随模式，立即应用跟随变换
    if (FOLLOW_CONFIG.enabled) {
      setTimeout(() => {
        const transform = this.calculateFollowTransform(
          this.screenWidth / 2, 
          this.screenHeight / 2
        );
        this.applyMapTransform(transform, false);
      }, 100);
    }

    // 连接WebSocket
    this.initWebSocket();
  },

  // 计算跟随视角的变换参数
  calculateFollowTransform(robotRealX, robotRealY, forceCalculate = false) {
    if (!forceCalculate && !this.data.followMode) {
      return this.data.mapTransform;
    }

    const { screenCenter } = this.data;
    const { centerOffset } = FOLLOW_CONFIG;
    
    // 计算目标中心点（考虑偏移）
    const targetCenterX = screenCenter.x + screenCenter.x * centerOffset.x;
    const targetCenterY = screenCenter.y + screenCenter.y * centerOffset.y;
    
    // 计算需要的平移量，使机器人位置对应到目标中心点
    // 变换原点在左上角(0,0)，所以：
    // 屏幕上的位置 = (原始位置 * 缩放) + 平移
    // 要让机器人显示在目标位置，需要：
    // 平移 = 目标位置 - (机器人原始位置 * 缩放)
    const scale = FOLLOW_CONFIG.followScale;
    const translateX = targetCenterX - robotRealX * scale;
    const translateY = targetCenterY - robotRealY * scale;
    
    console.log(`跟随计算详情:`);
    console.log(`- 屏幕中心: (${screenCenter.x}, ${screenCenter.y})`);
    console.log(`- 目标中心: (${targetCenterX}, ${targetCenterY})`);
    console.log(`- 机器人位置: (${robotRealX}, ${robotRealY})`);
    console.log(`- 缩放: ${scale}`);
    console.log(`- 计算平移: (${translateX.toFixed(1)}, ${translateY.toFixed(1)})`);
    
    return {
      translateX: translateX,
      translateY: translateY,
      scale: scale
    };
  },

  // 应用地图变换
  applyMapTransform(transform, smooth = true) {
    if (smooth && FOLLOW_CONFIG.smoothTransition) {
      // 使用动画过渡
      this.setData({
        mapTransform: transform
      });
    } else {
      // 立即应用变换
      this.setData({
        mapTransform: transform
      });
    }
  },

  // 切换跟随模式
  toggleFollowMode(e) {
    console.log('toggleFollowMode 被调用，事件:', e);
    console.log('当前跟随模式状态:', this.data.followMode);
    
    const followMode = !this.data.followMode;
    
    if (followMode) {
      // 启用跟随模式
      this.setData({ followMode });
      
      // 使用setTimeout确保setData完成后再计算变换
      setTimeout(() => {
        const transform = this.calculateFollowTransform(
          this.data.robotPosition.x, 
          this.data.robotPosition.y,
          true  // 强制计算，忽略followMode状态检查
        );
        this.applyMapTransform(transform);
        
        console.log('跟随模式已开启，当前机器人位置:', this.data.robotPosition);
        console.log('应用的变换:', transform);
      }, 50);
      
      wx.showToast({
        title: '跟随模式已开启',
        icon: 'success'
      });
    } else {
      // 关闭跟随模式
      this.setData({ followMode });
      
      // 立即重置视角
      this.resetMapView();
      
      // 确保变换被应用
      setTimeout(() => {
        console.log('重置后的变换:', this.data.mapTransform);
        // 如果变换没有正确应用，强制重置
        if (this.data.mapTransform.scale !== 1 || 
            this.data.mapTransform.translateX !== 0 || 
            this.data.mapTransform.translateY !== 0) {
          console.log('强制重置变换');
          this.setData({
            mapTransform: {
              translateX: 0,
              translateY: 0,
              scale: 1
            }
          });
        }
      }, 200);
      
      wx.showToast({
        title: '跟随模式已关闭',
        icon: 'success'
      });
    }
  },

  // 重置地图视角
  resetMapView() {
    console.log('重置地图视角');
    
    const resetTransform = {
      translateX: 0,
      translateY: 0,
      scale: 1
    };
    
    console.log('应用重置变换:', resetTransform);
    
    // 立即应用重置变换
    this.setData({
      mapTransform: resetTransform
    });
  },





  // 初始化WebSocket连接
  initWebSocket() {
    RECONNECT_CONFIG.currentRetry = 0;
    this.isReconnecting = false;
    this.connectWebSocket();
  },

  // 连接WebSocket
  connectWebSocket() {
    if (this.isReconnecting) {
      console.log('正在重连中，跳过本次连接');
      return;
    }

    if (RECONNECT_CONFIG.currentRetry >= RECONNECT_CONFIG.maxRetries) {
      console.error('达到最大重试次数，停止重连');
      return;
    }

    // 如果已经有连接，先关闭
    if (this.socketTask) {
      try {
        this.socketTask.close();
        this.socketTask = null;
      } catch (e) {
        console.error('关闭连接时发生错误:', e);
      }
    }

    console.log('正在连接WebSocket:', WS_URL);
    this.isReconnecting = true;

    // 创建WebSocket连接
    this.socketTask = wx.connectSocket({
      url: WS_URL,
      header: {
        'content-type': 'application/json'
      },
      success: (res) => {
        console.log('WebSocket连接创建成功:', res);
      },
      fail: (error) => {
        console.error('WebSocket连接创建失败:', error);
        this.handleReconnect();
      },
      complete: (res) => {
        console.log('WebSocket连接创建完成:', res);
      }
    });

    // 监听连接打开
    this.socketTask.onOpen((res) => {
      console.log('WebSocket连接已打开:', res);
      this.isReconnecting = false;
      RECONNECT_CONFIG.currentRetry = 0;
      this.setData({ connected: true });
    });

    // 监听消息
    this.socketTask.onMessage((res) => {
      console.log('收到WebSocket消息:', res.data);
      try {
        // 尝试解析消息
        let data = res.data;
        if (typeof data === 'string') {
          data = JSON.parse(data);
        }

        // 处理消息
        if (data.type === 'position') {
          // 处理位置更新
          this.handlePositionUpdate(data);
        } else if (data.type === 'alert') {
          // 处理告警信息
          this.handleAlertUpdate(data);
        } else {
          console.log('未知的消息类型:', data);
        }
      } catch (error) {
        console.error('处理WebSocket消息失败:', error);
      }
    });

    // 监听连接关闭
    this.socketTask.onClose((res) => {
      console.log('WebSocket连接已关闭:', res);
      this.setData({ connected: false });
      this.socketTask = null;

      // 如果不是主动关闭，则尝试重连
      if (!this.isManualClosed) {
        this.handleReconnect();
      }
    });

    // 监听错误
    this.socketTask.onError((res) => {
      console.error('WebSocket错误:', res);
      this.setData({ connected: false });
      
      // 如果发生错误，尝试重连
      if (!this.isManualClosed) {
        this.handleReconnect();
      }
    });
  },

  // 处理重连
  handleReconnect() {
    if (this.isManualClosed) {
      console.log('连接已手动关闭，不进行重连');
      return;
    }

    RECONNECT_CONFIG.currentRetry++;
    this.isReconnecting = false;

    const delay = Math.min(
      RECONNECT_CONFIG.retryDelay * Math.pow(1.5, RECONNECT_CONFIG.currentRetry - 1),
      RECONNECT_CONFIG.maxDelay
    );

    console.log(`将在 ${delay}ms 后进行第 ${RECONNECT_CONFIG.currentRetry} 次重连`);

    setTimeout(() => {
      if (!this.isManualClosed) {
        this.connectWebSocket();
      }
    }, delay);
  },

  // 处理位置更新
  handlePositionUpdate(data) {
    console.log('处理位置更新:', data);
    try {
      // 解析data字段中的JSON字符串
      let positionData;
      if (typeof data.data === 'string') {
        positionData = JSON.parse(data.data);
      } else {
        positionData = data.data;
      }

      // 确保positionData包含有效的x和y值
      if (!positionData || typeof positionData.x === 'undefined' || typeof positionData.y === 'undefined') {
        console.error('无效的位置数据:', positionData);
        return;
      }

      const robotX = Number(positionData.x);
      const robotY = Number(positionData.y);

      // 更新机器人位置
      this.setData({
        robotPosition: {
          x: robotX,
          y: robotY
        }
      });

      // 如果启用跟随模式，更新地图视角
      if (this.data.followMode) {
        const transform = this.calculateFollowTransform(robotX, robotY);
        this.applyMapTransform(transform);
      }

      console.log('机器人位置已更新:', this.data.robotPosition);
    } catch (error) {
      console.error('处理位置数据失败:', error);
    }
  },

  // 处理告警更新
  handleAlertUpdate(data) {
    try {
      // 解析data字段中的JSON字符串
      let alertData;
      if (typeof data.data === 'string') {
        alertData = JSON.parse(data.data);
      } else {
        alertData = data.data;
      }

      // 获取告警类型配置
      const alertType = ALERT_TYPES[alertData.type];
      if (!alertType) {
        console.error('未知的告警类型:', alertData.type);
        return;
      }

      // 生成唯一的告警ID
      const alertId = Date.now();

      // 创建新的告警标记
      const newAlert = {
        id: alertId,
        x: Number(alertData.x),
        y: Number(alertData.y),
        type: alertType.name,  // 使用中文名称
        visible: true,
        opacity: 1,
        color: alertType.color
      };

      // 添加新告警到列表
      this.setData({
        alerts: [...this.data.alerts, newAlert]
      });

      // 开始闪烁动画
      this.startAlertAnimation(alertId);

      // 5秒后移除告警
      setTimeout(() => {
        this.removeAlert(alertId);
      }, 3000);

    } catch (error) {
      console.error('处理告警数据失败:', error);
    }
  },

  // 开始告警闪烁动画
  startAlertAnimation(alertId) {
    let isVisible = true;
    const animationInterval = 200; // 闪烁间隔（毫秒）
    const duration = 3000; // 总持续时间（毫秒）
    const startTime = Date.now();

    const animate = () => {
      const currentTime = Date.now();
      if (currentTime - startTime >= duration) {
        return; // 动画结束
      }

      // 更新告警可见性
      isVisible = !isVisible;
      this.setData({
        alerts: this.data.alerts.map(alert => {
          if (alert.id === alertId) {
            return {
              ...alert,
              visible: isVisible
            };
          }
          return alert;
        })
      });

      // 继续下一帧动画
      setTimeout(animate, animationInterval);
    };

    // 开始动画
    animate();
  },

  // 移除告警
  removeAlert(alertId) {
    this.setData({
      alerts: this.data.alerts.filter(alert => alert.id !== alertId)
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
  zoomIn(e) {
    console.log('zoomIn 被调用，事件:', e);
    const currentTransform = this.data.mapTransform;
    const newScale = Math.min(currentTransform.scale + 0.3, 4.0);
    
    this.applyMapTransform({
      ...currentTransform,
      scale: newScale
    });
  },
  
  // 地图缩放：缩小
  zoomOut(e) {
    console.log('zoomOut 被调用，事件:', e);
    const currentTransform = this.data.mapTransform;
    const newScale = Math.max(currentTransform.scale - 0.3, 0.5);
    
    this.applyMapTransform({
      ...currentTransform,
      scale: newScale
    });
  },
  
  // 地图缩放：重置
  resetMap(e) {
    console.log('resetMap 被调用，事件:', e);
    this.resetMapView();
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
    if (e.touches.length === 1 && this.data.lastTouchPoint) {
      const deltaX = e.touches[0].pageX - this.data.lastTouchPoint.x;
      const deltaY = e.touches[0].pageY - this.data.lastTouchPoint.y;
      
      // 更新地图变换
      const currentTransform = this.data.mapTransform;
      this.applyMapTransform({
        ...currentTransform,
        translateX: currentTransform.translateX + deltaX,
        translateY: currentTransform.translateY + deltaY
      }, false); // 拖动时不使用平滑过渡
      
      this.setData({
        lastTouchPoint: {
          x: e.touches[0].pageX,
          y: e.touches[0].pageY
        }
      });
      
      console.log(`地图移动: ${deltaX}, ${deltaY}`);
    }
  },
  
  // 触摸结束事件
  touchEnd() {
    this.setData({ lastTouchPoint: null });
  },
  
  onUnload() {
    this.isManualClosed = true;
    
    if (this.socketTask) {
      try {
        this.socketTask.close();
      } catch (e) {
        console.error('关闭WebSocket连接时发生异常:', e);
      }
      this.socketTask = null;
    }
  }
}) 