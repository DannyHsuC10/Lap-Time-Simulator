# Vehicle_state 欄位與 track_speed_map_lap_data 計算來源

本文說明 `Vehicle_status.py` 中 `Vehicle_state` 的所有欄位，以及目前 `plot_track_speed_map.py` 匯出 `track_speed_map_lap_data.pkl` 時各欄位的計算方式。

## 座標與輪位順序

- 四輪陣列順序：`[FL, FR, RL, RR]`
- 車身座標：`x` 向前、`y` 向左、`z` 向上
- `theta` 順序：`[roll_f, roll_r, pitch, yaw]`
- 簡化剛性模型不解車體扭轉，所以 `roll_f = roll_r = roll_angle`
- aero modal 姿態：`qh = heave`、`qp = pitch`、`qr = (roll_f + roll_r) / 2`

## 時間

- `t`：累積時間，由距離增量 `ds` 和相鄰速度平均值積分得到。
- `dt`：與前一筆 sample 的時間差；第一筆為 `0.0`。

## 車身位置、速度、加速度

- `s`：車身座標位置 `[track_s, 0, heave]`。
  - `track_s` 來自賽道累積距離。
  - `heave` 來自 `sus_load.wheel_load_stiffness(..., return_state=True)` 的簡化剛性姿態估算。
- `v`：車身速度 `[speed, 0, heave_rate]`。
  - `speed` 來自 speed plan。
  - `heave_rate` 對 `heave(t)` 做時間微分。
- `a`：車身加速度 `[ax, ay, az]`。
  - `ax` 由速度對距離的變化計算：`(v2^2 - v1^2) / (2 ds)`。
  - `ay = speed^2 * curvature`。
  - `az` 對 `heave_rate(t)` 做時間微分。

## World frame

- `s_world`：世界座標位置 `[track_x, track_y, 0]`，由賽道幾何直接取得。
- `v_world`：將 body-frame `v` 依 `yaw` 旋轉到世界座標。
- `a_world`：將 body-frame `a` 依 `yaw` 旋轉到世界座標。

## 姿態、角速度、角加速度

- `theta`：`[roll_f, roll_r, pitch, yaw]`。
  - `roll_f`、`roll_r` 來自簡化剛性模型，且兩者相同。
  - `pitch` 來自簡化剛性模型。
  - `yaw` 由賽道中心線切線方向計算。
- `omega`：對 `theta(t)` 做時間微分。
- `alpha`：對 `omega(t)` 做時間微分。
- `M`：外部輸入力矩，目前 speed-map export 不做完整力矩平衡，保持預設 `0`。

## 四角簧上/簧下垂向狀態

- `zs`：由 `Vehicle_state.body_to_corner(s, theta, car)` 將 CG heave 與姿態轉成四角簧上位移。
- `vzs`：由 `body_to_corner(v, omega, car)` 轉成四角簧上速度。
- `azs`：由 `body_to_corner(a, alpha, car)` 轉成四角簧上加速度。
- `zu`、`vzu`、`azu`：簧下位移/速度/加速度。speed-map export 不是 transient suspension simulation，目前保持預設值。
- `zr`、`vzr`：路面位移/速度輸入。speed-map export 目前沒有路面 profile，保持預設值。

## 輪胎與輪載

- `Fz`：四輪垂直載荷，來自 `wheel_load_stiffness()` 的 `[FL, FR, RL, RR]`。
- `N`：目前等同 `Fz`，代表輪胎正向力。
- `Fs`、`Ft`：在完整懸吊模型中分別代表簧上/簧下相關力；speed-map export 目前以輪載填入，作為穩態等效值。
- `loaded_radius`：`car.R_tire - Fz / car.K_tire`，下限為 `0`。
- `omega_w`：`speed / loaded_radius`。
- `alpha_w`：車輪角加速度。speed-map export 目前沒有輪速 transient，保持預設值。
- `IA`：inclination/camber angle。speed-map export 目前未解 camber，保持預設值。
- `steer`：前輪用 bicycle model 幾何估算，後輪為 `0`。
  - 目前公式為 `atan((car.l * curvature) / (1 - car.lr * curvature))`。
  - 四輪欄位中前輪 `[FL, FR]` 使用相同 `steer_angle`，後輪 `[RL, RR]` 為 `0`。
- `SA`、`SL`：slip angle / slip ratio。
  - speed-map export 會使用 `Pacejka_tire_model/tire_force_map.pkl` 反查。
  - 對每一輪，先用當下 `Fz` 建立 tire map 2D slice，再以目前 `Fx/Fy` 需求在 `FX/FY` 點雲中找最近的 map 點，取出該點的 `SA_deg` 與 `SL`。
  - 右輪/左輪的 Fy 與 SA 符號會依 `Steady_tire_port.Four_wheel_output()` 的座標轉換處理。
- `tire_map_Fx_error`、`tire_map_Fy_error`：反查得到的 map 點與目標 `Fx/Fy` 的差值，用來檢查最近點反查品質。
- `Mx_w`、`Mz_w`、`RRT_w`：輪胎 overturning moment、aligning moment、rolling resistance torque。
  - speed-map export 會在反查 `SA/SL` 時，同步從 `tire_force_map.pkl` 取出 `MX/MZ/RRT`。
  - `Mz_w` 的右輪符號會反轉，`Mx_w` 的左輪符號會反轉，與 `Steady_tire_port.Four_wheel_output()` 的四輪座標轉換一致。
  - `RRT_w` 以 tire map 輸出的 rolling resistance torque 紀錄，四輪順序同樣為 `[FL, FR, RL, RR]`。

## 輪胎平面力

- `Fx`：四輪縱向力。
  - `Fx` 代表輪胎需要提供給車身的縱向力；除了 `mass * ax`，目前也會加入空氣阻力與滾動阻力補償。
  - 空氣阻力使用 aero model 依當下姿態得到的 `aero_CD` / `aero_F_drag`。
  - 滾動阻力由 tire map 的 `RRT_w / loaded_radius` 換算成等效縱向阻力。
  - 因此等速時若 `ax = 0`，仍會因 aero drag 與 rolling resistance 產生正的 `Fx`、`T_drive` 與 `gas_cmd`。
  - 加速時依 `car.T_rate` 固定比例分配最大可用馬達輸出，例如 `[0.67, 0.67, 1.0, 1.0]` 代表前輪最多只取最大輪端扭矩的 67%。
  - 若任一輪依固定比例輸出會超過該輪 tire-map Fx capacity，整體輸出會依最小允許比例一起降低，避免單輪超出負荷。
  - 煞車時先由 `power_brake_sys/brake.py` 使用 `car.F_driver`、`car.PR`、balance bar、master cylinder、caliper 和 disc 參數預先算出四輪最大煞車扭矩。
  - 煞車分配比例由煞車機構固定；若任一輪依固定比例煞車會超過該輪 tire-map Fx capacity，四輪煞車會一起依最小允許比例降低。
  - hold 等速時若需要抵消 aero drag 或 rolling resistance，會以驅動力固定比例分配並產生對應 `gas_cmd`。
- `Fy`：總側向力 `mass * ay` 依四輪輪載比例分配。
- `F_cg`：目前填入 `[mass * ax, sum(Fy), -sum(Fz)]`，代表車身運動所需的 CG 等效力；aero drag 與 rolling resistance 補償會反映在輪胎 `Fx` 與扭矩命令中。

## 空力

- `aero_qh`：給 aero model 的 heave modal coordinate。
- `aero_qp`：給 aero model 的 pitch modal coordinate。
- `aero_qr`：給 aero model 的 roll modal coordinate。
- `aero_F_down`：姿態回饋後的總下壓力。
- `aero_F_drag`：姿態回饋後的阻力。
- `aero_CL`：姿態回饋後的 lift coefficient。
- `aero_CD`：姿態回饋後的 drag coefficient。
- `COP`：`[COP_x, COP_y, cop_z]`。
- `aero_COP`：目前等同 `COP`，保留給 viewer 或後處理明確讀 aero 欄位。
- `F_aero`：`[-aero_F_drag, 0, -aero_F_down]`。
- `aero_force`：目前等同 `F_aero`。

## 懸吊 KC / hydraulic modal forces

這些欄位在完整 transient suspension model 中由 `Suspension_model_v13.py` 計算。speed-map export 目前只做穩態輪載與姿態估算，沒有解 hydraulic/KC transient，因此保持預設值。

- `F_heave_f`
- `F_heave_r`
- `F_roll_f`
- `F_roll_r`
- `F_roll_c`
- `F_warp`
- `F_corner`

## 控制命令與扭矩

- `gas_cmd`：`0~1` 的油門命令，`1` 代表目前車速下依 `car.T_rate` 固定分配的最大馬達輸出，`0` 代表鬆開。
  - 加速或等速補償阻力時，由實際需求總驅動力 `sum(Fx)` 除以固定比例下的最大可用驅動力計算，並限制在 `0~1`。
  - 若輪胎極限使最大輸出必須依最弱輪縮小，speed planner 產生的需求力也會跟著降低，因此 `gas_cmd` 會反映這個縮放後的實際命令。
- `brake_cmd`：`0~1` 的煞車命令，`1` 代表以 `car.F_driver` 踩到底後通過 `brake.py` 固定機構比例得到的最大煞車輸出，`0` 代表鬆開。
  - 煞車時由實際需求總煞車力 `-sum(Fx)` 除以固定比例下的最大可用煞車力計算，並限制在 `0~1`。
  - 若輪胎極限使最大煞車必須依最弱輪縮小，speed planner 產生的需求力也會跟著降低，因此 `brake_cmd` 會反映這個縮放後的實際命令。
- `phase`：由 speed plan 的加速度分類而來，值為 `accelerate`、`brake` 或 `hold`。
- `T_drive`：`max(Fx * loaded_radius, 0)`；加速與等速阻力補償時已反映 `car.T_rate` 固定分配與最弱輪縮放限制。
- `T_brake`：`min(Fx * loaded_radius, 0)`；煞車時已反映 `brake.py` 固定前後機構比例、`car.F_driver` 最大踏力限制與最弱輪縮放限制。
- `T_motor`：`T_drive / car.train_rate / car.gear_eta`。

## 目前仍是預設或近似的欄位

以下欄位在 speed-map export 中尚未由完整物理模型解出，主要是因為目前流程是 lap speed planner，不是逐時間步的 full vehicle transient simulation：

- `zu`、`vzu`、`azu`
- `zr`、`vzr`
- `alpha_w`
- `IA`
- `M`
- `F_heave_f`、`F_heave_r`、`F_roll_f`、`F_roll_r`、`F_roll_c`、`F_warp`、`F_corner`

後續如果要讓這些欄位更真實，需要在 speed plan 後新增更細的 per-sample tire inverse solve，或直接接 full transient vehicle model。
