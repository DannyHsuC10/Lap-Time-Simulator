# Lap-Time-Simulator

這個資料夾是一組 FSAE/方程式賽車 lap time 與 vehicle dynamics 模擬工具。整體看起來不是單一成熟套件，而是由 MATLAB 原型、簡化驗證模型、Python 快速迭代模型，以及一個簡單控制展示場共同組成。主線目標是把賽道幾何、車輛參數、輪胎 Magic Formula、空力、懸吊、動力與煞車限制整合起來，產生速度圖、lap time 與車輛狀態資料。

## 專案結構

```text
Lap-Time-Simulator/
├─ Control_Sim/              # Pygame 即時車輛控制 MVP
├─ Fast Iteration_Model/     # 目前最活躍的 Python 快速迭代模型
├─ TTR_sim/                  # 較完整的 MATLAB lap-time / vehicle model 原型
└─ TTR_simplify_sim/         # 簡化版 MATLAB 模型，用於較快驗證流程
```

## 主要子專案

### `Fast Iteration_Model/`

Python 版本是目前最像「整合中主模型」的部分。它把車輛系統拆成多個模組，並用 `plot_track_speed_map.py` 產生基於賽道曲率、輪胎 Fx/Fy envelope、空力載荷、動力與煞車限制的速度計畫。

重點檔案與資料夾：

- `car.py`：集中定義車輛參數，例如軸距、輪距、CG、質量、慣量、輪胎半徑/剛性、馬達傳動、煞車、空力、懸吊 KC 與懸吊幾何。
- `Vehicle_status.py`：車輛狀態容器，包含四輪位移/速度/力、輪速、slip angle/slip ratio、body 姿態、CG 運動、world frame 狀態與 JSON 儲存/載入。
- `plot_track_speed_map.py`：目前最完整的 lap speed map 腳本。流程是讀取 segment track、載入 tire force map、建立空力/輪胎載荷限制、做前向加速與後向煞車 pass，輸出 `track_speed_map.png` 與 `track_speed_map_lap_data.pkl`。
- `track/`：賽道幾何。`track_segments.py` 讀 segment CSV，將 LINE/ARC 轉成 section geometry；`track_geo.py` 提供一般 CSV racing line 與 circular track 幾何工具。
- `Pacejka_tire_model/`：Python 版 Magic Formula 6.12 輪胎模型。`model/api.py` 載入 `.tir` 並統一求解 `FX/FY/MX/MZ/RRT`；`Steady_tire_port.py`、`Transient_tire_port.py` 包裝四輪輸出；`tire_mapping_tool.py` 會建立/儲存輪胎力查表。
- `Suspension_sys/`：懸吊與輪載模型。`Suspension_model_v13.py` 做簧上/簧下、heave/roll/warp modal force、幾何力與 body dynamics；`KC_sys/` 定義 spring/damper/motion-ratio/KC 組合；`sus_geo/` 定義 roll center、anti、camber 與幾何傳力；`steady_state/sus_load.py` 提供穩態輪載轉移估算。
- `aero/`：`Aero.py` 是 modal aero model，依 heave/pitch/roll 和速度計算 downforce、drag、四輪空力分配與 COP。
- `power_brake_sys/`：馬達扭矩、煞車扭矩、traction controller、馬達曲線工具與 Excel 原始資料。
- `data/`：模擬紀錄、視覺化、STL 車輛/輪胎模型與已存狀態。`Simulation_Logger.py` 用 pickle 儲存 state time history。

目前 Python 專案的 `pyproject.toml` 宣告：

- Python `>=3.14`
- `matplotlib`
- `openpyxl`
- `pandas`
- `scipy`

常用執行方式：

```powershell
cd "Fast Iteration_Model"
uv sync
uv run python plot_track_speed_map.py
```

執行後預期會讀：

- `track/fsae_A_2025.csv`
- `Pacejka_tire_model/tire_force_map.pkl`
- `power_brake_sys/excel/motor_speed_vs_torque.xlsx`

並輸出：

- `track_speed_map.png`
- `track_speed_map_lap_data.pkl`

### `TTR_sim/`

這是比較完整的 MATLAB 原型。`TTR_lap.m` 是主要 lap simulation 腳本，會把 track、power、Vehicle_Dynamics、braking、MF62 tire model、vehicle data、aero 等資料夾加入 path，然後執行分段式空間域模擬。

重點流程：

1. 讀取車輛、空力、輪胎、馬達、煞車模型。
2. 從 `track/input_map.xlsx` 載入 segment，透過 `build_track.m` 建立完整賽道。
3. 用 `find_track_features.m` 找出彎道/曲率特徵。
4. `section_sim.m` 在每個 section 中判斷加速或煞車，呼叫 `vehicle_model.m` 取得當下縱向加速度。
5. 繪製 s-v、t-s、t-v、accel，以及賽道速度 heat map。
6. 儲存 `lap_result.mat`，內含 track、feature、car、aero、simulation history 與 summary。

其他入口：

- `TTR_GGV.m`：掃描速度與側向加速度，建立 GGV 曲面。
- `TTD_drag.m`：直線加速/煞車模擬。
- `TTR_skidpad.m`：skidpad 模擬。
- `lim_speed.m`：依曲率搜尋最大穩定速度。

### `TTR_simplify_sim/`

這是 MATLAB 簡化版，結構與 `TTR_sim` 類似，但車輛模型與資料夾比較少，主要集中在：

- `TTR_lap.m`
- `TTR_GGV.m`
- `TTR_drag.m`
- `TTR_skidpad.m`
- `section_sim.m`
- `track/`
- `vechiles/`

它適合用來快速檢查 lap-time pipeline、賽道建置、section 加減速邏輯與結果輸出，不需要完整 MATLAB 版本中較複雜的 Vehicle_Dynamics、MF62、aero/braking/power 子資料夾拆分。

### `Control_Sim/`

這是一個小型 Pygame 控制 MVP，使用簡化 bicycle model 來即時控制車輛。

重點檔案：

- `vehicle.py`：定義 `VehicleParameters`、`VehicleState`、鍵盤輸入、bicycle model 更新與畫面繪製。
- `vehicle_mvp.py`：Pygame loop，方向鍵控制油門、煞車與轉向。
- `main.py`：目前只是 `Hello from by-key!` 的 uv 範本入口。

注意：`pyproject.toml` 目前沒有列出 `pygame`，但 `vehicle.py` 和 `vehicle_mvp.py` 需要它。

可能執行方式：

```powershell
cd Control_Sim
uv add pygame
uv run python vehicle_mvp.py
```

## 模型資料流理解

目前最清楚的整合方向如下：

```text
Track CSV / input_map.xlsx
        ↓
賽道中心線、曲率、section、邊界
        ↓
車輛參數 Car / Vehicle_data
        ↓
輪胎 Magic Formula + tire map
        ↓
空力 downforce / drag / COP
        ↓
懸吊與輪載轉移
        ↓
動力與煞車扭矩限制
        ↓
每個賽道位置的 lateral / drive / brake limit
        ↓
forward acceleration pass + backward braking pass
        ↓
speed map、lap time、state log、圖表
```

Python 版目前在 `plot_track_speed_map.py` 中已經把這條資料流串得最完整；MATLAB 版則保留了更早期的完整車輛模型驗證流程。

## 目前狀態與注意事項

- 根目錄目前沒有測試架構；多數驗證是透過腳本直接跑圖、輸出 `.mat`、`.pkl` 或 `.png`。
- `Fast Iteration_Model/main.py` 與 `Control_Sim/main.py` 都還是 uv 初始化範本，不是實際模擬入口。
- `Fast Iteration_Model/README.md` 在目前 Git 狀態中是刪除狀態；本 README 是新增在根目錄，用來描述整個資料夾。
- 專案中有 `__pycache__`、`.venv`、`.mat`、`.pkl`、`.png` 等生成或環境檔案，之後若要整理版本控制，建議補 `.gitignore`。
- `vechiles` 資料夾名稱目前沿用既有拼法，README 也照實描述，不改路徑。

## 建議下一步

1. 決定主線是否以 `Fast Iteration_Model/plot_track_speed_map.py` 作為 Python lap-time pipeline 入口。
2. 補上 `.gitignore`，避免 cache、venv、輸出圖與大型中間資料混進版本控制。
3. 將 Python 依賴補齊，尤其是 `Control_Sim` 的 `pygame`，以及任何視覺化/STL 工具需要的套件。
4. 把 MATLAB 與 Python 的車輛參數來源對齊，避免 `Vehicle_data.m`、`car.py`、Excel/TIR 檔各自漂移。
5. 為 tire map、track loading、speed plan 建立小型 smoke test，至少確認主要輸入檔存在且主流程可以跑到輸出。
