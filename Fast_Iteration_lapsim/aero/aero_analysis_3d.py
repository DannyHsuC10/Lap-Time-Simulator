"""
空力模型 3D 分析
分析 CoP (Center of Pressure) 与 roll/pitch 的关系
分析下压力与 roll/pitch 的关系
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# 导入空力模型
from Aero import ModalAeroModel

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

current_dir = Path(__file__).resolve().parent


def save_current_figure(filename):
    output_path = current_dir / filename
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"[OK] saved: {output_path}")


class AnalysisState:

    def __init__(self, speed, heave, pitch, roll):
        self.s = np.array([0.0, 0.0, heave])
        self.v = np.array([speed, 0.0, 0.0])
        self.theta = np.array([roll, roll, pitch, 0.0])

# ============================
# 建立 Aero Model
# ============================
aero = ModalAeroModel()

# ============================
# 参数扫描
# ============================
V = 16.7  # m/s (固定速度)

# Pitch and roll state angle range (rad)
pitch_range = np.deg2rad(np.linspace(-3.0, 3.0, 30))
roll_range = np.deg2rad(np.linspace(-3.0, 3.0, 30))
heave = -0.01

# 创建网格
pitch_grid, roll_grid = np.meshgrid(pitch_range, roll_range)
pitch_plot = np.degrees(pitch_grid)
roll_plot = np.degrees(roll_grid)
pitch_axis = np.degrees(pitch_range)
roll_axis = np.degrees(roll_range)

# 初始化数据数组
F_down_grid = np.zeros_like(pitch_grid)
F_FL_grid = np.zeros_like(pitch_grid)     # 新增：前左
F_FR_grid = np.zeros_like(pitch_grid)     # 新增：前右
F_RL_grid = np.zeros_like(pitch_grid)     # 新增：后左
F_RR_grid = np.zeros_like(pitch_grid)     # 新增：后右
COP_x_grid = np.zeros_like(pitch_grid)    # 修改：纵向 CoP
COP_y_grid = np.zeros_like(pitch_grid)    # 新增：横向 CoP
CL_grid = np.zeros_like(pitch_grid)
CD_grid = np.zeros_like(pitch_grid)
F_drag_grid = np.zeros_like(pitch_grid)
Mz_aero_grid = np.zeros_like(pitch_grid)  # 横滚力矩

# ============================
# 计算空力特性
# ============================
for i, pitch in enumerate(pitch_range):
    for j, roll in enumerate(roll_range):
        state = AnalysisState(V, heave, pitch, roll)
        
        # 计算空力
        result = aero.calculate_from_state(state)
        
        F_down_grid[j, i] = result["F_down"]
        F_FL_grid[j, i] = result["F_FL"]
        F_FR_grid[j, i] = result["F_FR"]
        F_RL_grid[j, i] = result["F_RL"]
        F_RR_grid[j, i] = result["F_RR"]
        CL_grid[j, i] = result["CL"]
        CD_grid[j, i] = result["CD"]
        F_drag_grid[j, i] = result["F_drag"]
        COP_x_grid[j, i] = result["COP_x"]      # 纵向 CoP
        COP_y_grid[j, i] = result["COP_y"]      # 横向 CoP
        
        # 横滚力矩（左右下压力不对称）
        dF_roll = (result["F_FL"] + result["F_RL"]) - (result["F_FR"] + result["F_RR"])
        Mz_aero_grid[j, i] = dF_roll * (aero.track / 2)

# ============================
# 3D 绘图 - Figure 1: CoP vs Pitch & Roll
# ============================
fig = plt.figure(figsize=(16, 12))

# 子图 1: CoP_x (纵向) 3D 曲面
ax1 = fig.add_subplot(2, 3, 1, projection='3d')
surf1 = ax1.plot_surface(
    pitch_plot,
    roll_plot,
    COP_x_grid,
    cmap='RdYlBu_r',
    alpha=0.8,
    edgecolor='none'
)
ax1.set_xlabel('Pitch (deg)')
ax1.set_ylabel('Roll (deg)')
ax1.set_zlabel('CoP_x (m)')
ax1.set_title('Longitudinal CoP vs Pitch & Roll', fontsize=11, fontweight='bold')
ax1.view_init(elev=20, azim=45)
fig.colorbar(surf1, ax=ax1, shrink=0.5, label='CoP_x (m)')

# 子图 2: CoP_y (横向) 3D 曲面
ax2 = fig.add_subplot(2, 3, 2, projection='3d')
surf2 = ax2.plot_surface(
    pitch_plot,
    roll_plot,
    COP_y_grid,
    cmap='RdYlBu_r',
    alpha=0.8,
    edgecolor='none'
)
ax2.set_xlabel('Pitch (deg)')
ax2.set_ylabel('Roll (deg)')
ax2.set_zlabel('CoP_y (m)')
ax2.set_title('Lateral CoP vs Pitch & Roll', fontsize=11, fontweight='bold')
ax2.view_init(elev=20, azim=45)
fig.colorbar(surf2, ax=ax2, shrink=0.5, label='CoP_y (m)')

# 子图 3: CoP_y 等高线图
ax3 = fig.add_subplot(2, 3, 3)
contour = ax3.contourf(pitch_plot, roll_plot, COP_y_grid, levels=20, cmap='RdYlBu_r')
ax3.contour(pitch_plot, roll_plot, COP_y_grid, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax3.set_xlabel('Pitch (deg)')
ax3.set_ylabel('Roll (deg)')
ax3.set_title('Lateral CoP Contour Map', fontsize=11, fontweight='bold')
ax3.grid(True, alpha=0.3)
fig.colorbar(contour, ax=ax3, label='CoP_y (m)')

# 子图 4: Pitch 截面 - CoP_x (Roll = 0)
ax4 = fig.add_subplot(2, 3, 4)
roll_idx = len(roll_range) // 2  # Roll = 0 的索引
ax4.plot(pitch_axis, COP_x_grid[roll_idx, :], 'b-', linewidth=2.5, label='CoP_x')
ax4.axhline(y=0, color='r', linestyle='--', linewidth=1.5, alpha=0.5, label='Centerline')
ax4.set_xlabel('Pitch (deg)')
ax4.set_ylabel('CoP_x (m)')
ax4.set_title('Longitudinal CoP vs Pitch\n(Roll=0°)', fontsize=11, fontweight='bold')
ax4.grid(True, alpha=0.3)
ax4.legend()

# 子图 5: Roll 截面 - CoP_y (Pitch = 0)
ax5 = fig.add_subplot(2, 3, 5)
pitch_idx = len(pitch_range) // 2  # Pitch = 0 的索引
ax5.plot(roll_axis, COP_y_grid[:, pitch_idx], 'g-', linewidth=2.5, label='CoP_y')
ax5.axhline(y=0, color='r', linestyle='--', linewidth=1.5, alpha=0.5, label='Centerline')
ax5.set_xlabel('Roll (deg)')
ax5.set_ylabel('CoP_y (m)')
ax5.set_title('Lateral CoP vs Roll\n(Pitch=0°)', fontsize=11, fontweight='bold')
ax5.grid(True, alpha=0.3)
ax5.legend()

# 子图 6: CoP_x 等高线图
ax6 = fig.add_subplot(2, 3, 6)
contour2 = ax6.contourf(pitch_plot, roll_plot, COP_x_grid, levels=20, cmap='viridis')
ax6.contour(pitch_plot, roll_plot, COP_x_grid, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax6.set_xlabel('Pitch (deg)')
ax6.set_ylabel('Roll (deg)')
ax6.set_title('Longitudinal CoP Contour Map', fontsize=11, fontweight='bold')
ax6.grid(True, alpha=0.3)
fig.colorbar(contour2, ax=ax6, label='CoP_x (m)')

plt.tight_layout()
save_current_figure('CoP_analysis_3d.png')

# ============================
# 3D 绘图 - Figure 2: 前后四轮下压力分布
# ============================
fig = plt.figure(figsize=(16, 12))

# 子图 1: 总下压力 3D 曲面
ax1 = fig.add_subplot(2, 3, 1, projection='3d')
surf1 = ax1.plot_surface(
    pitch_plot,
    roll_plot,
    F_down_grid,
    cmap='viridis',
    alpha=0.8,
    edgecolor='none'
)
ax1.set_xlabel('Pitch (deg)')
ax1.set_ylabel('Roll (deg)')
ax1.set_zlabel('Downforce (N)')
ax1.set_title('Total Aerodynamic Downforce\nvs Pitch & Roll', fontsize=11, fontweight='bold')
ax1.view_init(elev=20, azim=45)
fig.colorbar(surf1, ax=ax1, shrink=0.5, label='Downforce (N)')

# 子图 2: 前轮总下压力 (F_FL + F_FR) 3D
ax2 = fig.add_subplot(2, 3, 2, projection='3d')
F_front_total = F_FL_grid + F_FR_grid
surf2 = ax2.plot_surface(
    pitch_plot,
    roll_plot,
    F_front_total,
    cmap='cool',
    alpha=0.8,
    edgecolor='none'
)
ax2.set_xlabel('Pitch (deg)')
ax2.set_ylabel('Roll (deg)')
ax2.set_zlabel('Front Downforce (N)')
ax2.set_title('Front Axle Downforce\nvs Pitch & Roll', fontsize=11, fontweight='bold')
ax2.view_init(elev=20, azim=45)
fig.colorbar(surf2, ax=ax2, shrink=0.5, label='Front (N)')

# 子图 3: 后轮总下压力 (F_RL + F_RR) 3D
ax3 = fig.add_subplot(2, 3, 3, projection='3d')
F_rear_total = F_RL_grid + F_RR_grid
surf3 = ax3.plot_surface(
    pitch_plot,
    roll_plot,
    F_rear_total,
    cmap='autumn',
    alpha=0.8,
    edgecolor='none'
)
ax3.set_xlabel('Pitch (deg)')
ax3.set_ylabel('Roll (deg)')
ax3.set_zlabel('Rear Downforce (N)')
ax3.set_title('Rear Axle Downforce\nvs Pitch & Roll', fontsize=11, fontweight='bold')
ax3.view_init(elev=20, azim=45)
fig.colorbar(surf3, ax=ax3, shrink=0.5, label='Rear (N)')

# 子图 4: 左侧总下压力 (F_FL + F_RL) vs 右侧 (F_FR + F_RR)
ax4 = fig.add_subplot(2, 3, 4)
F_left = F_FL_grid + F_RL_grid
F_right = F_FR_grid + F_RR_grid
dF_LR = F_left - F_right  # 左右不对称
contour4 = ax4.contourf(pitch_plot, roll_plot, dF_LR, levels=20, cmap='RdBu')
ax4.contour(pitch_plot, roll_plot, dF_LR, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax4.set_xlabel('Pitch (deg)')
ax4.set_ylabel('Roll (deg)')
ax4.set_title('Left-Right Downforce Asymmetry\n(positive = left larger)', fontsize=11, fontweight='bold')
ax4.grid(True, alpha=0.3)
fig.colorbar(contour4, ax=ax4, label='F_left - F_right (N)')

# 子图 5: Front/Rear 比例
ax5 = fig.add_subplot(2, 3, 5)
contour5 = ax5.contourf(pitch_plot, roll_plot, F_drag_grid, levels=20, cmap='magma')
ax5.contour(pitch_plot, roll_plot, F_drag_grid, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax5.set_xlabel('Pitch (deg)')
ax5.set_ylabel('Roll (deg)')
ax5.set_title('Aerodynamic Drag Force', fontsize=11, fontweight='bold')
ax5.grid(True, alpha=0.3)
fig.colorbar(contour5, ax=ax5, label='Drag (N)')

# 子图 6: 总下压力等高线图
ax6 = fig.add_subplot(2, 3, 6)
contour6 = ax6.contourf(pitch_plot, roll_plot, F_down_grid, levels=20, cmap='viridis')
ax6.contour(pitch_plot, roll_plot, F_down_grid, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax6.set_xlabel('Pitch (deg)')
ax6.set_ylabel('Roll (deg)')
ax6.set_title('Total Downforce Contour Map', fontsize=11, fontweight='bold')
ax6.grid(True, alpha=0.3)
fig.colorbar(contour6, ax=ax6, label='Downforce (N)')

plt.tight_layout()
save_current_figure('downforce_analysis_3d.png')

# ============================
# 3D 绘图 - Figure 3: 四轮下压力分布
# ============================
fig = plt.figure(figsize=(16, 12))

# 子图 1: 前左下压力
ax1 = fig.add_subplot(2, 3, 1, projection='3d')
surf1 = ax1.plot_surface(pitch_plot, roll_plot, F_FL_grid, cmap='cool', alpha=0.8, edgecolor='none')
ax1.set_xlabel('Pitch (deg)')
ax1.set_ylabel('Roll (deg)')
ax1.set_zlabel('F_FL (N)')
ax1.set_title('Front Left Downforce', fontsize=11, fontweight='bold')
ax1.view_init(elev=20, azim=45)
fig.colorbar(surf1, ax=ax1, shrink=0.5)

# 子图 2: 前右下压力
ax2 = fig.add_subplot(2, 3, 2, projection='3d')
surf2 = ax2.plot_surface(pitch_plot, roll_plot, F_FR_grid, cmap='cool', alpha=0.8, edgecolor='none')
ax2.set_xlabel('Pitch (deg)')
ax2.set_ylabel('Roll (deg)')
ax2.set_zlabel('F_FR (N)')
ax2.set_title('Front Right Downforce', fontsize=11, fontweight='bold')
ax2.view_init(elev=20, azim=45)
fig.colorbar(surf2, ax=ax2, shrink=0.5)

# 子图 3: 后左下压力
ax3 = fig.add_subplot(2, 3, 3, projection='3d')
surf3 = ax3.plot_surface(pitch_plot, roll_plot, F_RL_grid, cmap='autumn', alpha=0.8, edgecolor='none')
ax3.set_xlabel('Pitch (deg)')
ax3.set_ylabel('Roll (deg)')
ax3.set_zlabel('F_RL (N)')
ax3.set_title('Rear Left Downforce', fontsize=11, fontweight='bold')
ax3.view_init(elev=20, azim=45)
fig.colorbar(surf3, ax=ax3, shrink=0.5)

# 子图 4: 后右下压力
ax4 = fig.add_subplot(2, 3, 4, projection='3d')
surf4 = ax4.plot_surface(pitch_plot, roll_plot, F_RR_grid, cmap='autumn', alpha=0.8, edgecolor='none')
ax4.set_xlabel('Pitch (deg)')
ax4.set_ylabel('Roll (deg)')
ax4.set_zlabel('F_RR (N)')
ax4.set_title('Rear Right Downforce', fontsize=11, fontweight='bold')
ax4.view_init(elev=20, azim=45)
fig.colorbar(surf4, ax=ax4, shrink=0.5)

# 子图 5: 前轮左右差异 (F_FL - F_FR)
ax5 = fig.add_subplot(2, 3, 5)
dF_front = F_FL_grid - F_FR_grid
contour5 = ax5.contourf(pitch_plot, roll_plot, dF_front, levels=20, cmap='RdBu')
ax5.contour(pitch_plot, roll_plot, dF_front, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax5.set_xlabel('Pitch (deg)')
ax5.set_ylabel('Roll (deg)')
ax5.set_title('Front Axle L-R Asymmetry\n(F_FL - F_FR)', fontsize=11, fontweight='bold')
ax5.grid(True, alpha=0.3)
fig.colorbar(contour5, ax=ax5, label='ΔF (N)')

# 子图 6: 后轮左右差异 (F_RL - F_RR)
ax6 = fig.add_subplot(2, 3, 6)
dF_rear = F_RL_grid - F_RR_grid
contour6 = ax6.contourf(pitch_plot, roll_plot, dF_rear, levels=20, cmap='RdBu')
ax6.contour(pitch_plot, roll_plot, dF_rear, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax6.set_xlabel('Pitch (deg)')
ax6.set_ylabel('Roll (deg)')
ax6.set_title('Rear Axle L-R Asymmetry\n(F_RL - F_RR)', fontsize=11, fontweight='bold')
ax6.grid(True, alpha=0.3)
fig.colorbar(contour6, ax=ax6, label='ΔF (N)')

plt.tight_layout()
save_current_figure('four_wheel_downforce_3d.png')

# ============================
# 3D 绘图 - Figure 4: CL 与 CoP 的关系
# ============================
fig = plt.figure(figsize=(14, 10))

# 子图 1: CL 3D 曲面
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
surf1 = ax1.plot_surface(
    pitch_plot,
    roll_plot,
    CL_grid,
    cmap='RdBu',
    alpha=0.8,
    edgecolor='none'
)
ax1.set_xlabel('Pitch (deg)')
ax1.set_ylabel('Roll (deg)')
ax1.set_zlabel('Lift Coefficient')
ax1.set_title('Lift Coefficient vs Pitch & Roll', fontsize=11, fontweight='bold')
ax1.view_init(elev=20, azim=45)
fig.colorbar(surf1, ax=ax1, shrink=0.5, label='CL')

# 子图 2: CL 等高线
ax2 = fig.add_subplot(2, 2, 2)
contour = ax2.contourf(pitch_plot, roll_plot, CL_grid, levels=20, cmap='RdBu')
ax2.contour(pitch_plot, roll_plot, CL_grid, levels=10, colors='black', linewidths=0.5, alpha=0.3)
ax2.set_xlabel('Pitch (deg)')
ax2.set_ylabel('Roll (deg)')
ax2.set_title('CL Contour Map', fontsize=11, fontweight='bold')
ax2.grid(True, alpha=0.3)
fig.colorbar(contour, ax=ax2, label='CL')

# 子图 3: CoP 与 CL 的关系（参数化曲线）
ax3 = fig.add_subplot(2, 2, 3)
scatter = ax3.scatter(
    CL_grid.flatten(),
    COP_x_grid.flatten(),
    c=pitch_plot.flatten(),
    s=30,
    cmap='coolwarm',
    alpha=0.6,
    edgecolors='black',
    linewidth=0.5
)
ax3.set_xlabel('Lift Coefficient (CL)')
ax3.set_ylabel('CoP_x (m)')
ax3.set_title('Longitudinal CoP vs CL (color: Pitch in deg)', fontsize=11, fontweight='bold')
ax3.grid(True, alpha=0.3)
fig.colorbar(scatter, ax=ax3, label='Pitch (deg)')

# 子图 4: Pitch 依赖性分析
ax4 = fig.add_subplot(2, 2, 4)
for roll_val in [-2, -1, 0, 1, 2]:  # deg
    roll_idx = np.argmin(np.abs(roll_axis - roll_val))
    ax4.plot(pitch_axis, COP_x_grid[roll_idx, :], 
             marker='o', markersize=4, label=f'Roll={roll_val}deg', linewidth=2)
nominal_cop_x = aero.L * (aero.front_balance - 0.5)
ax4.axhline(y=nominal_cop_x, color='k', linestyle='--', linewidth=1.5, alpha=0.5, label='Nominal')
ax4.set_xlabel('Pitch (deg)')
ax4.set_ylabel('CoP_x (m)')
ax4.set_title('Longitudinal CoP Sensitivity to Pitch', fontsize=11, fontweight='bold')
ax4.grid(True, alpha=0.3)
ax4.legend(fontsize=9)

plt.tight_layout()
save_current_figure('cl_cop_analysis_3d.png')

# ============================
# 生成统计摘要
# ============================
print("\n" + "="*80)
print("AERODYNAMIC MODEL ANALYSIS SUMMARY")
print("="*80)
print(f"\n测试条件:")
print(f"  速度: {V} m/s ({V*3.6:.1f} km/h)")
print(f"  Heave: {heave*1000:.1f} mm")
print(f"  Pitch 範圍: {np.degrees(pitch_range.min()):.1f} ~ {np.degrees(pitch_range.max()):.1f} deg")
print(f"  Roll 範圍: {np.degrees(roll_range.min()):.1f} ~ {np.degrees(roll_range.max()):.1f} deg")

print(f"\n下压力统计:")
max_idx = np.unravel_index(np.argmax(F_down_grid), F_down_grid.shape)
min_idx = np.unravel_index(np.argmin(F_down_grid), F_down_grid.shape)
print(f"  最大下压力: {F_down_grid.max():.2f} N (在 Pitch={np.degrees(pitch_grid[max_idx]):.1f}deg, Roll={np.degrees(roll_grid[max_idx]):.1f}deg)")
print(f"  最小下压力: {F_down_grid.min():.2f} N (在 Pitch={np.degrees(pitch_grid[min_idx]):.1f}deg, Roll={np.degrees(roll_grid[min_idx]):.1f}deg)")
print(f"  平均下压力: {F_down_grid.mean():.2f} N")
print(f"  下压力变化: {F_down_grid.max() - F_down_grid.min():.2f} N")

print(f"\n空气阻力统计:")
print(f"  最大阻力: {F_drag_grid.max():.2f} N")
print(f"  最小阻力: {F_drag_grid.min():.2f} N")
print(f"  平均阻力: {F_drag_grid.mean():.2f} N")
print(f"  CD 范围: {CD_grid.min():.4f} ~ {CD_grid.max():.4f}")

print(f"\nCoP 统计:")
print(f"  CoP_x (纵向) 范围: {COP_x_grid.min():.4f} ~ {COP_x_grid.max():.4f} m")
print(f"  CoP_y (横向) 范围: {COP_y_grid.min():.4f} ~ {COP_y_grid.max():.4f} m")
print(f"  CoP_x 平均值: {COP_x_grid.mean():.4f} m")
print(f"  CoP_y 平均值: {COP_y_grid.mean():.4f} m")

print(f"\n四轮下压力统计:")
print(f"  F_FL 范围: {F_FL_grid.min():.2f} ~ {F_FL_grid.max():.2f} N")
print(f"  F_FR 范围: {F_FR_grid.min():.2f} ~ {F_FR_grid.max():.2f} N")
print(f"  F_RL 范围: {F_RL_grid.min():.2f} ~ {F_RL_grid.max():.2f} N")
print(f"  F_RR 范围: {F_RR_grid.min():.2f} ~ {F_RR_grid.max():.2f} N")

F_front_mean = (F_FL_grid + F_FR_grid).mean()
F_rear_mean = (F_RL_grid + F_RR_grid).mean()
print(f"\n前后轴平均分布:")
print(f"  前轴平均: {F_front_mean:.2f} N")
print(f"  后轴平均: {F_rear_mean:.2f} N")
print(f"  前/后比例: {F_front_mean / (F_rear_mean + 1e-6):.4f}")

print(f"\nPitch 灵敏度 (dCoP/dPitch at Roll=0):")
roll_idx = len(roll_range) // 2
cop_x_at_roll = COP_x_grid[roll_idx, :]
cop_y_at_roll = COP_y_grid[roll_idx, :]
pitch_vals = pitch_range
pitch_sensitivity_x = np.polyfit(pitch_vals, cop_x_at_roll, 1)[0]
pitch_sensitivity_y = np.polyfit(pitch_vals, cop_y_at_roll, 1)[0]
print(f"  dCoP_x/dPitch: {pitch_sensitivity_x:.4f} m/rad")
print(f"  dCoP_y/dPitch: {pitch_sensitivity_y:.4f} m/rad")

print(f"\nRoll 灵敏度 (dCoP/dRoll at Pitch=0):")
pitch_idx = len(pitch_range) // 2
cop_x_at_pitch = COP_x_grid[:, pitch_idx]
cop_y_at_pitch = COP_y_grid[:, pitch_idx]
roll_vals = roll_range
roll_sensitivity_x = np.polyfit(roll_vals, cop_x_at_pitch, 1)[0]
roll_sensitivity_y = np.polyfit(roll_vals, cop_y_at_pitch, 1)[0]
print(f"  dCoP_x/dRoll: {roll_sensitivity_x:.4f} m/rad")
print(f"  dCoP_y/dRoll: {roll_sensitivity_y:.4f} m/rad")

print("\n" + "="*80)
print("[OK] 所有分析图已生成!")
print("  - CoP_analysis_3d.png (CoP纵向和横向分析)")
print("  - downforce_analysis_3d.png (总下压力和前后分析)")
print("  - four_wheel_downforce_3d.png (四轮下压力详细分析)")
print("  - cl_cop_analysis_3d.png (CL与CoP关系)")
print("="*80 + "\n")

plt.show()

