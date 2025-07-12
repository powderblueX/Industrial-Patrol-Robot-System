import matplotlib.pyplot as plt
import numpy as np

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# 不确定性变化率数据
uncertainty_rates = [-15, -10, -5, 0, 5, 10, 15]

# 各类别的IRR数据（转换为百分比数值）
operating_income_irr = [16.21, 24.86, 34.89, 42.80, 52.05, 61.44, 70.97]
construction_investment_irr = [50.81, 48.02, 45.35, 42.80, 40.36, 38.01, 35.76]
operation_maintenance_irr = [45.94, 44.92, 43.81, 42.80, 41.79, 40.77, 39.76]
personnel_costs_irr = [46.37, 45.18, 43.99, 42.80, 41.15, 39.46, 37.88]

# 创建图形
plt.figure(figsize=(12, 8))

# 绘制四条折线
plt.plot(uncertainty_rates, operating_income_irr, 'o-', linewidth=2, markersize=6, 
         color='#1f77b4', label='营业收入 (Operating Income)')
plt.plot(uncertainty_rates, construction_investment_irr, 's-', linewidth=2, markersize=6, 
         color='#ff7f0e', label='建设投资 (Construction Investment)')
plt.plot(uncertainty_rates, operation_maintenance_irr, '^-', linewidth=2, markersize=6, 
         color='#2ca02c', label='运营维护成本 (Operation and Maintenance Costs)')
plt.plot(uncertainty_rates, personnel_costs_irr, 'd-', linewidth=2, markersize=6, 
         color='#d62728', label='人员成本 (Personnel Costs)')

# 设置图形属性
plt.xlabel('不确定性变化率 (%)', fontsize=12, fontweight='bold')
plt.ylabel('内部收益率 IRR (%)', fontsize=12, fontweight='bold')
plt.title('不确定性分析：各因素变化率对IRR的影响', fontsize=14, fontweight='bold', pad=20)

# 设置网格
plt.grid(True, alpha=0.3, linestyle='--')

# 设置坐标轴范围
plt.xlim(-16, 16)
plt.ylim(15, 75)

# 添加图例
plt.legend(loc='best', fontsize=10, frameon=True, shadow=True)

# 设置坐标轴刻度
plt.xticks(uncertainty_rates)
plt.yticks(np.arange(15, 76, 5))

# 添加零线参考
plt.axvline(x=0, color='gray', linestyle=':', alpha=0.7)
plt.axhline(y=42.80, color='gray', linestyle=':', alpha=0.7)

# 调整布局
plt.tight_layout()

# 显示图形
plt.show()

# 保存图形
plt.savefig('irr_uncertainty_analysis.png', dpi=300, bbox_inches='tight')
print("图形已保存为 'irr_uncertainty_analysis.png'") 