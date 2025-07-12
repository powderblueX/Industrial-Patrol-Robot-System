"use strict";

// history.js
const app = getApp();
Page({
  data: {
    // 搜索和筛选
    searchText: '',
    selectedMonth: '',
    selectedMonthText: '',
    selectedTypeIndex: null,
    selectedTypeName: '',
    selectedLevelIndex: null,
    selectedLevelName: '',
    // 告警类型和级别选项
    alarmTypes: [{
      id: 'all',
      name: '全部'
    }, {
      id: 'obstacle',
      name: '障碍物检测'
    }, {
      id: 'fall',
      name: '摔倒检测'
    }, {
      id: 'intrusion',
      name: '入侵检测'
    }, {
      id: 'fire',
      name: '火灾检测'
    }, {
      id: 'smoke',
      name: '烟雾检测'
    }],
    alarmLevels: [{
      id: 'all',
      name: '全部'
    }, {
      id: 'high',
      name: '高'
    }, {
      id: 'medium',
      name: '中'
    }, {
      id: 'low',
      name: '低'
    }],
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
  onLoad: function () {
    this.loadMockData();
    this.applyFilters();
  },
  // 生成模拟数据
  loadMockData: function () {
    const alarmTypes = ['障碍物检测', '摔倒检测', '入侵检测', '火灾检测', '烟雾检测'];
    const alarmLevels = ['high', 'medium', 'low'];
    const levelTexts = {
      'high': '高',
      'medium': '中',
      'low': '低'
    };
    const mockAlarms = [];

    // 生成50条模拟数据
    for (let i = 0; i < 50; i++) {
      const type = alarmTypes[Math.floor(Math.random() * alarmTypes.length)];
      const level = alarmLevels[Math.floor(Math.random() * alarmLevels.length)];

      // 生成随机日期（过去30天内）
      const date = new Date();
      date.setDate(date.getDate() - Math.floor(Math.random() * 30));
      const time = this.formatTime(date);
      mockAlarms.push({
        id: 'alarm_' + (i + 1),
        type: type,
        level: level,
        levelText: levelTexts[level],
        time: time,
        description: `检测到${type}告警，请及时处理。`
      });
    }

    // 按时间降序排序
    mockAlarms.sort((a, b) => {
      return new Date(b.time) - new Date(a.time);
    });
    this.setData({
      alarms: mockAlarms
    });
  },
  // 格式化日期时间
  formatTime: function (date) {
    const year = date.getFullYear();
    const month = (date.getMonth() + 1).toString().padStart(2, '0');
    const day = date.getDate().toString().padStart(2, '0');
    const hours = date.getHours().toString().padStart(2, '0');
    const minutes = date.getMinutes().toString().padStart(2, '0');
    return `${year}-${month}-${day} ${hours}:${minutes}`;
  },
  // 搜索输入
  onSearchInput: function (e) {
    this.setData({
      searchText: e.detail.value
    });
  },
  // 搜索确认
  onSearchConfirm: function () {
    this.applyFilters();
  },
  // 清除搜索
  clearSearch: function () {
    this.setData({
      searchText: ''
    }, () => {
      this.applyFilters();
    });
  },
  // 月份选择
  onMonthChange: function (e) {
    const value = e.detail.value;
    if (value) {
      const [year, month] = value.split('-');
      this.setData({
        selectedMonth: value,
        selectedMonthText: `${year}年${month}月`
      });
    } else {
      this.setData({
        selectedMonth: '',
        selectedMonthText: ''
      });
    }
    this.applyFilters();
  },
  // 类型选择
  onTypeChange: function (e) {
    const index = e.detail.value;
    const selectedType = this.data.alarmTypes[index];
    this.setData({
      selectedTypeIndex: index,
      selectedTypeName: index > 0 ? selectedType.name : ''
    });
    this.applyFilters();
  },
  // 级别选择
  onLevelChange: function (e) {
    const index = e.detail.value;
    const selectedLevel = this.data.alarmLevels[index];
    this.setData({
      selectedLevelIndex: index,
      selectedLevelName: index > 0 ? selectedLevel.name : ''
    });
    this.applyFilters();
  },
  // 应用筛选条件
  applyFilters: function () {
    const {
      alarms,
      searchText,
      selectedMonth,
      selectedTypeIndex,
      selectedLevelIndex,
      alarmTypes,
      alarmLevels
    } = this.data;
    let filtered = alarms.concat([]);

    // 搜索文本筛选
    if (searchText) {
      filtered = filtered.filter(alarm => {
        return alarm.type.includes(searchText) || alarm.description.includes(searchText);
      });
    }

    // 月份筛选
    if (selectedMonth) {
      const [year, month] = selectedMonth.split('-');
      filtered = filtered.filter(alarm => {
        const alarmDate = new Date(alarm.time);
        return alarmDate.getFullYear() === parseInt(year) && alarmDate.getMonth() + 1 === parseInt(month);
      });
    }

    // 类型筛选
    if (selectedTypeIndex > 0) {
      const selectedType = alarmTypes[selectedTypeIndex].name;
      filtered = filtered.filter(alarm => alarm.type === selectedType);
    }

    // 级别筛选
    if (selectedLevelIndex > 0) {
      const selectedLevel = alarmLevels[selectedLevelIndex].id;
      filtered = filtered.filter(alarm => alarm.level === selectedLevel);
    }

    // 重置分页
    this.setData({
      filteredAlarms: filtered.slice(0, this.data.pageSize),
      currentPage: 1,
      hasMoreData: filtered.length > this.data.pageSize
    });
  },
  // 加载更多
  loadMoreAlarms: function () {
    if (!this.data.hasMoreData || this.data.isLoading) return;
    this.setData({
      isLoading: true
    });
    setTimeout(() => {
      const {
        filteredAlarms,
        alarms,
        searchText,
        selectedMonth,
        selectedTypeIndex,
        selectedLevelIndex,
        pageSize,
        currentPage,
        alarmTypes,
        alarmLevels
      } = this.data;
      let filtered = alarms.concat([]);

      // 应用筛选条件
      if (searchText) {
        filtered = filtered.filter(alarm => {
          return alarm.type.includes(searchText) || alarm.description.includes(searchText);
        });
      }
      if (selectedMonth) {
        const [year, month] = selectedMonth.split('-');
        filtered = filtered.filter(alarm => {
          const alarmDate = new Date(alarm.time);
          return alarmDate.getFullYear() === parseInt(year) && alarmDate.getMonth() + 1 === parseInt(month);
        });
      }
      if (selectedTypeIndex > 0) {
        const selectedType = alarmTypes[selectedTypeIndex].name;
        filtered = filtered.filter(alarm => alarm.type === selectedType);
      }
      if (selectedLevelIndex > 0) {
        const selectedLevel = alarmLevels[selectedLevelIndex].id;
        filtered = filtered.filter(alarm => alarm.level === selectedLevel);
      }
      const nextPage = currentPage + 1;
      const nextPageSize = pageSize * nextPage;
      this.setData({
        filteredAlarms: filtered.slice(0, nextPageSize),
        currentPage: nextPage,
        hasMoreData: filtered.length > nextPageSize,
        isLoading: false
      });
    }, 500);
  },
  // 显示告警详情
  showAlarmDetail: function (e) {
    const index = e.currentTarget.dataset.index;
    const alarm = this.data.filteredAlarms[index];
    this.setData({
      showDetailModal: true,
      currentAlarm: alarm
    });
  },
  // 隐藏告警详情
  hideAlarmDetail: function () {
    this.setData({
      showDetailModal: false,
      currentAlarm: null
    });
  },
  // 导航到地图页面
  navigateToMap: function () {
    wx.switchTab({
      url: '/pages/map/map'
    });
  }
});