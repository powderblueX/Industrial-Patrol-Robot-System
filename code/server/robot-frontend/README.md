# 机器人巡检小程序

这是一个用于机器人巡检的微信小程序，主要功能包括：

1. 实时地图显示
   - 通过WebSocket接收机器人实时位置并在地图上显示
   - **动态视角跟随**：地图视角自动跟随机器人移动并放大显示周围区域
   - 接收告警信息并在地图上标记
   - 点击告警标记可查看详情

2. 历史告警查询
   - 按时间范围查询历史告警记录
   - 分页加载告警数据
   - 查看告警详情

## 🎯 动态视角跟随功能

### 核心特性
- **自动跟随**：地图视角始终跟随机器人位置
- **智能缩放**：自动放大到2倍显示机器人周围详细区域
- **平滑过渡**：视角切换使用平滑动画效果
- **手动控制**：支持手动拖拽和缩放，5秒后自动恢复跟随
- **视野指示**：机器人周围显示脉冲视野圈

### 操作说明
1. **跟随模式开关**：点击右下角🎯按钮开启/关闭跟随模式
2. **手动控制**：
   - 拖拽地图：临时进入手动控制模式
   - 缩放操作：使用+/-按钮调整缩放级别
   - 重置视角：点击↺按钮重置到全局视图
3. **状态指示**：
   - 右上角显示当前模式（跟随模式/手动控制中）
   - 左上角显示连接状态

### 配置参数
```javascript
const FOLLOW_CONFIG = {
  enabled: true,           // 默认启用跟随模式
  followScale: 2.0,        // 跟随时的缩放倍数
  smoothTransition: true,  // 启用平滑过渡动画
  transitionDuration: 300, // 过渡动画时长
  centerOffset: {          // 机器人在屏幕中的位置偏移
    x: 0,                  // 水平偏移（0=中心）
    y: -0.1                // 垂直偏移（-0.1=偏上10%）
  }
}
```

## 配置说明

1. WebSocket服务器配置
   - 在`pages/map/map.js`中修改`socketUrl`为您的WebSocket服务器地址

2. 历史告警API配置
   - 在`pages/history/history.js`中修改API请求地址

3. 跟随模式配置
   - 在`pages/map/map.js`中修改`FOLLOW_CONFIG`调整跟随行为

## 接口数据格式

### WebSocket消息格式

1. 位置更新消息
```json
{
  "type": "position",
  "data": {
    "x": 100,
    "y": 200,
    "mapWidth": 1000,
    "mapHeight": 800
  }
}
```

2. 告警消息
```json
{
  "type": "alarm",
  "data": {
    "id": "alarm_001",
    "level": "high", // high, medium, low
    "type": "温度异常",
    "location": "A区设备间",
    "description": "温度超过阈值，请及时处理",
    "position": {
      "x": 300,
      "y": 400
    },
    "mapWidth": 1000,
    "mapHeight": 800,
    "time": "2023-06-12 15:30:45"
  }
}
```

### 历史告警API响应格式

```json
{
  "code": 0,
  "message": "success",
  "alarms": [
    {
      "id": "alarm_001",
      "level": "high",
      "type": "温度异常",
      "location": "A区设备间",
      "description": "温度超过阈值，请及时处理",
      "time": "2023-06-12 15:30:45",
      "handledStatus": "handled", // handled, unhandled
      "handledBy": "张工程师",
      "handledTime": "2023-06-12 16:20:30"
    }
    // ...更多告警数据
  ]
}
```

## 资源文件

1. 图片资源
   - `/images/map.png` - 平面图
   - `/images/robot.png` - 机器人图标
   - `/images/alarm.png` - 告警图标

2. 音频资源
   - `/audio/alarm.mp3` - 告警提示音

## 开发注意事项

1. 确保已在微信开发者工具中开启"增强编译"
2. 在真机测试WebSocket连接时，确保手机和服务器在同一网络环境
3. 后端API未就绪时，可启用模拟数据进行测试（见历史告警页面JS文件）
4. 跟随模式在机器人移动频繁时效果最佳
5. 可根据实际地图尺寸调整`followScale`参数以获得最佳显示效果 