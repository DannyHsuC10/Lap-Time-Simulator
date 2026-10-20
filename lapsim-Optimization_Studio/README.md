# LapSim Optimization Studio

這個子專案以穩態圈速模擬與車輛參數最佳化為主。相較 `Steady_lapsim`，它保留了較完整的輪胎、空力、動力、煞車與輪載計算，並增加圈速執行器、車輛慣量計算和兩種最佳化流程。

## 模擬流程

```text
track_parameter_optimizer.py 或 slalom_track_width_optimizer.py
                         ↓
              lap_simulation_runner.py
                         ↓
            theoretical_lap_simulator.py
                         ↓
賽道 + 車輛慣量 + 輪胎查表 + 空力 + 動力/煞車 + 穩態輪載
```

## 主要入口

- `benchmark_lap_simulation_runner.py`：執行一次圈速模擬並顯示時間與圈速。
- `track_parameter_optimizer.py`：最佳化車輛懸吊剛性參數。
- `slalom_track_width_optimizer.py`：以 slalom 路徑進行參數最佳化。
- `Pacejka_tire_model/tire_mapping_tool.py`：從 `.tir` 輪胎資料建立模擬所用的 `tire_force_map.pkl`。

執行前先在本目錄安裝依賴：

```powershell
uv sync
uv run python benchmark_lap_simulation_runner.py
```

## 與 `Steady_lapsim` 的差異

`Steady_lapsim` 是較直接的穩態圈速模型，檔案與流程較少。Optimization Studio 的新增部分是最佳化器、最佳化用執行器、slalom 路徑分析，以及從簧上/簧下質量表計算慣量的模組；這些都與其最佳化用途直接相關，因此保留。

本次整理移除了不在圈速或最佳化流程中的舊動態懸吊模型、懸吊資料管理/備份工具、STL 與狀態資料視覺化工具、獨立空力/輪胎繪圖範例、舊馬達曲線擬合工具與其原始資料，以及已提交的 Python bytecode 快取。輸出目錄 `data/` 保留為模擬與最佳化圖檔的目的地。

## 保留的模型輸入

- `track/fsae_A_2025.csv`：賽道分段資料。
- `Inertia/Center_of_gravity_*.xlsx`：簧上與前/後簧下質量分布。
- `power_brake_sys/excel/motor_speed_vs_torque.xlsx`：馬達扭矩查表。
- `Pacejka_tire_model/tire_force_map.pkl`：圈速計算使用的輪胎力查表。
- `Pacejka_tire_model/tir/`：輪胎模型原始參數，供重新產生查表。

懸吊結構與 `Steady_lapsim` 對齊：`Suspension_sys/sus_load.py` 計算穩態輪載，車輛的前/後 heave 與 roll 剛性直接存在 `car.py`。舊 KC 元件庫不再參與這個簡化圈速模型。
