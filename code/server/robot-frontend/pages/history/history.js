// history.js
const app = getApp()

Page({
  data: {
    // 筛选条件
    selectedDate: '',
    selectedTypeIndex: null,
    selectedTypeName: '',
    
    // 告警类型选项
    alarmTypes: [
      { id: 'all', name: '全部' },
      { id: 'INTRUSION', name: '入侵' },
      { id: 'FIRE', name: '火灾' },
      { id: 'SMOKE', name: '烟雾' }
    ],
    
    // 告警数据
    alarms: [],
    filteredAlarms: [],
    
    // 分页加载
    pageSize: 10,
    currentPage: 1,
    hasMoreData: true,
    isLoading: false,
    
    // 弹窗
    showDetailModal: false,
    currentAlarm: null
  },

  onLoad: function() {
    this.loadAlarms();
  },

  // 从后端加载告警数据
  loadAlarms: function() {
    const that = this;
    const { selectedDate, selectedTypeIndex, alarmTypes } = this.data;
    
    // 构建查询参数
    let url = 'http://100.81.120.217:8080/api/alerts';
    const params = [];
    
    if (selectedDate) {
      params.push(`date=${selectedDate}`);
    }
    
    if (selectedTypeIndex > 0) {
      params.push(`type=${alarmTypes[selectedTypeIndex].id}`);
    }
    
    if (params.length > 0) {
      url += '?' + params.join('&');
    }

    wx.request({
      url: url,
      method: 'GET',
      success: function(res) {
        if (res.statusCode === 200 && Array.isArray(res.data)) {
          const alerts = res.data.map(alert => {
            // 获取告警类型的中文名称
            const typeName = that.getAlertTypeName(alert.type);
            
            // 格式化时间
            const timestamp = alert.timestamp ? new Date(alert.timestamp) : new Date();
            const time = that.formatTime(timestamp);
            
            // 构建告警描述
            const description = alert.description || `检测到${typeName}告警，请及时处理。`;
            
            return {
              id: alert.id || `alert_${Date.now()}_${Math.random()}`,
              type: typeName,
              time: time,
              description: description,
              // 保留原始数据，以备需要
              rawData: alert
            };
          });
          
          that.setData({ 
            alarms: alerts,
            filteredAlarms: alerts.slice(0, that.data.pageSize),
            hasMoreData: alerts.length > that.data.pageSize
          });
        } else {
          console.error('响应数据格式错误:', res);
          wx.showToast({
            title: '数据格式错误',
            icon: 'error'
          });
        }
      },
      fail: function(err) {
        console.error('请求失败：', err);
        wx.showToast({
          title: '网络错误',
          icon: 'error'
        });
      }
    });
  },

  // 获取告警类型的中文名称
  getAlertTypeName: function(type) {
    const alertType = this.data.alarmTypes.find(t => t.id === type);
    return alertType ? alertType.name : type;
  },
  
  // 格式化日期时间
  formatTime: function(date) {
    const year = date.getFullYear();
    const month = (date.getMonth() + 1).toString().padStart(2, '0');
    const day = date.getDate().toString().padStart(2, '0');
    const hours = date.getHours().toString().padStart(2, '0');
    const minutes = date.getMinutes().toString().padStart(2, '0');
    const seconds = date.getSeconds().toString().padStart(2, '0');
    
    return `${year}/${month}/${day} ${hours}:${minutes}:${seconds}`;
  },
  
  // 日期选择
  onDateChange: function(e) {
    const date = e.detail.value;
    this.setData({
      selectedDate: date
    }, () => {
      this.loadAlarms();
    });
  },
  
  // 类型选择
  onTypeChange: function(e) {
    const index = e.detail.value;
    const type = this.data.alarmTypes[index];
    
    this.setData({
      selectedTypeIndex: index,
      selectedTypeName: type.name
    }, () => {
      this.loadAlarms();
    });
  },
  
  // 加载更多
  loadMoreAlarms: function() {
    if (!this.data.hasMoreData || this.data.isLoading) return;
    
    this.setData({ isLoading: true });
    
    setTimeout(() => {
      const { alarms, pageSize, currentPage } = this.data;
      
      const nextPage = currentPage + 1;
      const nextPageSize = pageSize * nextPage;
      
      this.setData({
        filteredAlarms: alarms.slice(0, nextPageSize),
        currentPage: nextPage,
        hasMoreData: alarms.length > nextPageSize,
        isLoading: false
      });
    }, 500);
  },
  
  // 显示告警详情
  showAlarmDetail: function(e) {
    const index = e.currentTarget.dataset.index;
    const alarm = this.data.filteredAlarms[index];
    
    if (alarm) {
      console.log('显示告警详情:', alarm);
      this.setData({
        showDetailModal: true,
        currentAlarm: alarm
      });
    } else {
      console.error('未找到告警数据:', index);
    }
  },
  
  // 隐藏告警详情
  hideAlarmDetail: function() {
    this.setData({
      showDetailModal: false,
      currentAlarm: null
    });
  },

  // 导航到地图页面
  navigateToMap: function() {
    wx.switchTab({
      url: '/pages/map/map'
    });
  }
}) 