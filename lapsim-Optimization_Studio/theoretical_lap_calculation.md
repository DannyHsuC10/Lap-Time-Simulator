# Fast Iteration Model 理論單圈計算邏輯

本文說明 `Fast Iteration_Model` 目前如何由賽道、車輛、輪胎、空力、動力與煞車資料，計算理論最佳單圈速度分佈。主要入口是 `theoretical_lap_simulator.py`。

> 目前這套工具不是完整 transient optimal control，而是 steady-state / quasi-static 的理論速度圖。它假設車輛總是沿著賽道中心線行駛，並在每一個賽道離散點上使用輪胎 Fx/Fy 極限、空力、載重轉移、動力與煞車能力來找出可達速度。

## 1. 整體資料流

主要執行流程在 `main()`：

1. 讀取賽道分段 CSV：

   ```python
   segment_track = SegmentTrack(csv_file=DEFAULT_SEGMENT_CSV, track_width=3.0, ds=0.5)
   track = build_segment_speed_path(segment_track)
   ```

2. 建立車輛參數與動力/煞車限制：

   ```python
   car = Car()
   power_brake_limit = PowerBrakeSystemLimit(car)
   ```

3. 載入輪胎力 map：

   ```python
   tire_limit_map = AeroTireLimitMap.build(...)
   ```

4. 建立速度規劃：

   ```python
   speed_plan = build_friction_limited_speed_plan(...)
   ```

5. 計算 lap time、繪製 speed map、輸出 lap data：

   ```python
   planned_lap_time = lap_time_from_speed(track.s, speed_plan.speed)
   plot_speed_map(...)
   export_lap_log(...)
   ```

整體概念可以寫成：

$$
\text{Track} + \text{Car} + \text{Tire Map} + \text{Aero} + \text{Power/Brake}
\rightarrow v(s), a_x(s), F_{z,i}(s), F_{x,i}(s), F_{y,i}(s)
$$

其中：

- $s$：沿賽道中心線的距離
- $v(s)$：該位置的規劃車速
- $a_x(s)$：縱向加速度
- $F_{z,i}$、$F_{x,i}$、$F_{y,i}$：第 $i$ 顆輪胎的垂直、縱向、側向力

## 2. 賽道幾何與曲率

`SegmentTrack` 從 CSV 讀取每一段賽道，分成直線與圓弧。`build_segment_speed_path()` 將所有 section 展開成離散點：

$$
\left(x_j, y_j, s_j, \kappa_j\right), \quad j = 0,1,\dots,N-1
$$

對每一段：

- 直線：

  $$
  \kappa = 0
  $$

- 圓弧：

  $$
  \kappa = \frac{1}{R_\text{signed}}
  $$

其中 $R_\text{signed}$ 會依照左彎/右彎帶正負號。速度限制計算時通常使用：

$$
|\kappa|
$$

車輛在曲率 $\kappa$ 上以速度 $v$ 行駛時，需要的側向加速度為：

$$
a_y = v^2 \kappa
$$

## 3. 輪胎力 map

輪胎資料由 `Pacejka_tire_model/tire_force_map.pkl` 提供，透過 `TireForceMap` 查表。表格軸為：

$$
F_z,\quad \alpha,\quad \kappa_\text{slip}
$$

輸出包含：

$$
F_x,\quad F_y,\quad M_x,\quad M_z,\quad RRT
$$

也就是：

$$
\left(F_x,F_y,M_x,M_z,RRT\right)
= f_\text{tire}(F_z,\alpha,\kappa_\text{slip})
$$

### 3.1 純側向極限

在建立 `AeroTireLimitMap` 時，程式會對不同垂直載重 $F_z$ 找出純側向最大力：

$$
F_{y,\max}(F_z)
= \max_{\alpha}\left|F_y(F_z,\alpha,\kappa_\text{slip}=0)\right|
$$

這會形成：

$$
F_z \rightarrow F_{y,\max}
$$

供彎速限制使用。

### 3.2 Fx/Fy envelope

對每一個取樣載重 $F_z$，程式會在輪胎 map 的 $(F_x,F_y)$ 點雲上建立 convex hull：

$$
\mathcal{H}(F_z)=\text{ConvexHull}\left\{(F_x,F_y)\right\}
$$

當某一輪已經需要側向力 $F_y$ 時，程式會在 hull 邊界上找出可用的煞車與驅動縱向力：

$$
F_{x,\min}(F_z,F_y) \le F_x \le F_{x,\max}(F_z,F_y)
$$

其中：

- $F_{x,\min}$：煞車方向可用縱向力
- $F_{x,\max}$：驅動方向可用縱向力

這是工具同時考慮轉向、加速、煞車的核心。

## 4. 空力模型

空力模型使用 `ModalAeroModel`，以速度與車身模態姿態計算下壓力、阻力與 COP。

輸入模態為：

$$
q_h = \text{heave}, \quad q_p = \text{pitch}, \quad q_r = \text{roll}
$$

升力係數與阻力係數：

$$
C_L =
C_{L0}
 + K_h |q_h|
 + K_p |q_p|
 + K_r |q_r|
 + K_{hh} q_h^2
 + K_{pp} q_p^2
$$

$$
C_D =
C_{D0}
 + K_{d,h}|q_h|
 + K_{d,p}|q_p|
 + K_{d,r}|q_r|
$$

下壓力與阻力：

$$
F_\text{down}
=
-\frac{1}{2}\rho A v^2 C_L
$$

$$
F_\text{drag}
=
\frac{1}{2}\rho A v^2 C_D
$$

因為車輛設定中 $C_L$ 通常為負值，所以上式中的 $F_\text{down}$ 為正的下壓力。

前後分配：

$$
\lambda_f = \lambda_{f0} + K_{\text{bal},p} q_p
$$

$$
F_\text{front} = \lambda_f F_\text{down},
\quad
F_\text{rear} = (1-\lambda_f)F_\text{down}
$$

左右分配：

$$
\lambda_l = 0.5 + K_{\text{bal},r} q_r
$$

$$
F_{FL}=F_\text{front}\lambda_l,\quad
F_{FR}=F_\text{front}(1-\lambda_l)
$$

$$
F_{RL}=F_\text{rear}\lambda_l,\quad
F_{RR}=F_\text{rear}(1-\lambda_l)
$$

## 5. 輪重與車身姿態估算

輪重由 `wheel_load_stiffness()` 計算。這是一個穩態載重轉移模型，輸入為：

$$
m,\quad g,\quad F_\text{down},\quad a_x,\quad a_y,\quad h_{cg},\quad L,\quad t_f,\quad t_r
$$

以及前後 heave / roll stiffness。

總垂直載重：

$$
W = mg + F_\text{down}
$$

靜態前軸載重：

$$
F_{\text{front,static}} = W \frac{l_r}{L}
$$

後軸載重：

$$
F_{\text{rear,static}} = W - F_{\text{front,static}}
$$

其中：

$$
L = l_f + l_r
$$

### 5.1 縱向載重轉移

縱向加速度造成 pitch moment：

$$
M_\text{pitch} = m a_x h_{cg}
$$

以 heave stiffness 比例分配：

$$
K_\text{pitch} = K_{\text{heave},f} + K_{\text{heave},r}
$$

$$
M_{\text{pitch},f}
= M_\text{pitch}
\frac{K_{\text{heave},f}}{K_\text{pitch}}
$$

$$
M_{\text{pitch},r}
= M_\text{pitch}
\frac{K_{\text{heave},r}}{K_\text{pitch}}
$$

轉成前後軸載重變化：

$$
\Delta F_f = \frac{M_{\text{pitch},f}}{L},
\quad
\Delta F_r = \frac{M_{\text{pitch},r}}{L}
$$

程式中：

$$
F_\text{front}=F_{\text{front,static}}-\Delta F_f
$$

$$
F_\text{rear}=F_{\text{rear,static}}+\Delta F_r
$$

### 5.2 側向載重轉移

側向加速度造成 roll moment：

$$
M_\text{roll}=m a_y h_{cg}
$$

以前後 roll stiffness 比例分配：

$$
K_\text{roll}=K_{\text{roll},f}+K_{\text{roll},r}
$$

$$
M_{\text{roll},f}
=M_\text{roll}
\frac{K_{\text{roll},f}}{K_\text{roll}}
$$

$$
M_{\text{roll},r}
=M_\text{roll}
\frac{K_{\text{roll},r}}{K_\text{roll}}
$$

轉為左右輪載重差：

$$
\Delta F_{\text{roll},f}
=\frac{M_{\text{roll},f}}{t_f}
$$

$$
\Delta F_{\text{roll},r}
=\frac{M_{\text{roll},r}}{t_r}
$$

四輪輪重：

$$
F_{z,FL}=\frac{F_\text{front}}{2}-\Delta F_{\text{roll},f}
$$

$$
F_{z,FR}=\frac{F_\text{front}}{2}+\Delta F_{\text{roll},f}
$$

$$
F_{z,RL}=\frac{F_\text{rear}}{2}-\Delta F_{\text{roll},r}
$$

$$
F_{z,RR}=\frac{F_\text{rear}}{2}+\Delta F_{\text{roll},r}
$$

### 5.3 空力與姿態迭代

`AeroTireLimitMap.wheel_load_state()` 會做數次迭代：

1. 先用目前姿態估算空力：

   $$
   F_\text{down}, F_\text{drag}
   = f_\text{aero}(v,q_h,q_p,q_r)
   $$

2. 用空力、$a_x$、$a_y$ 算輪重與姿態。
3. 用新的 $q_h,q_p,q_r$ 再重新計算空力。

因此空力與載重不是完全固定查表，而是有簡化的姿態回授。

## 6. 側向速度上限

`build_aero_lateral_speed_profile()` 先找每個賽道點的側向速度上限。

對於直線或近似直線：

$$
|\kappa| \le \kappa_\text{floor}
\Rightarrow
v_\text{lat}=v_\text{max}
$$

對於彎道，程式用 binary search 找最大 $v$，使得：

$$
F_{y,\text{need,total}}
\le
\eta \sum_i F_{y,\max}(F_{z,i})
$$

其中：

$$
F_{y,\text{need,total}} = m v^2 \kappa
$$

$$
F_{z,i}=f_\text{load}(v,a_x=0,a_y=v^2\kappa)
$$

$$
\eta = \text{safety factor}
$$

也就是：

$$
v_\text{lat}(s)
=
\max v
\quad
\text{s.t.}
\quad
m v^2 |\kappa(s)|
\le
\eta \sum_i F_{y,\max}(F_{z,i})
$$

初始速度圖為：

$$
v_0(s)=v_\text{lat}(s)
$$

## 7. 縱向加速限制

`drive_accel_at_speed()` 用 binary search 尋找該速度、該曲率下最大可行正向加速度 $a_x$。

對候選值 $a_x$：

1. 先算側向需求：

   $$
   a_y = v^2\kappa
   $$

2. 算四輪輪重：

   $$
   F_{z,i}=f_\text{load}(v,a_x,a_y)
   $$

3. 將總側向力按輪重比例分給四輪：

   $$
   F_{y,i}
   =
   \frac{F_{z,i}}{\sum_j F_{z,j}}
   \left(m a_y\right)
   $$

4. 由輪胎 Fx/Fy envelope 查剩餘驅動力：

   $$
   F_{x,i}^{drive,max}
   =
   F_{x,\max}(F_{z,i},F_{y,i})
   $$

5. 由馬達與傳動計算動力系統可給的輪上扭矩：

   $$
   \omega_w=\frac{v}{R_\text{loaded}}
   $$

   $$
   RPM
   =
   \omega_w \cdot GR \cdot \frac{60}{2\pi}
   $$

   $$
   T_w
   =
   T_m(RPM)\cdot GR \cdot \eta_g
   $$

   $$
   F_{x,i}^{power}
   =
   \frac{T_{w,i}}{R_\text{loaded}}
   $$

6. 驅動力會被輪胎極限縮放：

   $$
   F_{x,i}^{drive}
   =
   \lambda F_{x,i}^{power}
   $$

   $$
   \lambda
   =
   \min_i
   \left(
   \frac{F_{x,i}^{drive,max}}{F_{x,i}^{power}}
   \right),
   \quad
   0\le \lambda \le 1
   $$

7. 扣掉空氣阻力與滾阻，得到可用加速度：

   $$
   a_{x,\text{cap}}
   =
   \frac{
   \sum_i F_{x,i}^{drive}
   -F_\text{drag}
   -F_\text{rr}
   }{m}
   $$

若：

$$
a_x \le a_{x,\text{cap}}
$$

則候選加速度可行，binary search 會往上找；否則往下找。

## 8. 煞車減速度限制

`brake_decel_at_speed()` 與加速邏輯相同，只是使用負向縱向力。

煞車系統由 `brake.py` 計算。踏板力經踏板比、balance bar、master cylinder、caliper 與 disc 產生輪上煞車扭矩。

master cylinder 力：

$$
F_{mc}=F_\text{driver} \cdot PR
$$

前後油壓：

$$
P_{mc,f}
=
\frac{F_{mc} \cdot bb_f}{A_{mc,f}}
$$

$$
P_{mc,r}
=
\frac{F_{mc} \cdot bb_r}{A_{mc,r}}
$$

卡鉗夾持力：

$$
F_{\text{caliper},f}
=
P_{mc,f}\mu_\text{disc}A_{\text{caliper},f}
$$

$$
F_{\text{caliper},r}
=
P_{mc,r}\mu_\text{disc}A_{\text{caliper},r}
$$

煞車扭矩：

$$
T_{\text{brake},f}=F_{\text{caliper},f}r_\text{disc}
$$

$$
T_{\text{brake},r}=F_{\text{caliper},r}r_\text{disc}
$$

輪胎可用煞車力由 envelope 得到：

$$
F_{x,i}^{brake,min}
=
F_{x,\min}(F_{z,i},F_{y,i})
$$

煞車力同樣會被輪胎極限縮放，最後可用減速度為：

$$
a_{x,\text{decel,cap}}
 =
\frac{
-\sum_i F_{x,i}^{brake}
+F_\text{drag}
+F_\text{rr}
}{m}
$$

注意煞車時，空氣阻力與滾阻會幫助減速，所以在上式中為正貢獻。

## 9. 前向/後向速度 pass

速度規劃核心在 `build_friction_limited_speed_plan()`。

初始：

$$
v_j = v_{\text{lat},j}
$$

### 9.1 前向加速 pass

從起點往終點掃描。相鄰點距離：

$$
\Delta s_j = s_{j+1}-s_j
$$

在第 $j$ 點計算可用加速度：

$$
a_{x,j}^{drive}=f_\text{drive}(v_j,\kappa_j)
$$

由運動學：

$$
v_{j+1,\text{reachable}}
=
\sqrt{v_j^2 + 2 a_{x,j}^{drive}\Delta s_j}
$$

下一點速度不能超過側向極限，也不能超過從上一點加速可達速度：

$$
v_{j+1}
=
\min
\left(
v_{j+1},
v_{j+1,\text{reachable}}
\right)
$$

這一步會限制彎道出口與直線上的加速能力。

### 9.2 後向煞車 pass

從終點往起點掃描。已知下一點速度 $v_{j+1}$，求第 $j$ 點最高能有多快，且能在 $\Delta s_j$ 內煞到 $v_{j+1}$。

程式用 binary search 找：

$$
v_{j,\max}^{brake}
$$

條件來自：

$$
v_{j+1}^2
\ge
v_j^2 - 2 a_{x,j}^{brake}\Delta s_j
$$

或寫成可行條件：

$$
v_j^2 - 2 a_{x,j}^{brake}\Delta s_j
\le
v_{j+1}^2
$$

最後更新：

$$
v_j
=
\min
\left(
v_j,
v_{j,\max}^{brake}
\right)
$$

這一步會把煞車區往彎前推回去。

### 9.3 多次 pass

程式預設做 3 次：

$$
\text{forward pass} \rightarrow \text{backward pass}
$$

反覆修正後，得到同時滿足：

- 彎中側向極限
- 出彎加速能力
- 入彎煞車能力
- 動力系統限制
- 煞車系統限制
- 輪胎 Fx/Fy 合成極限
- 空力與輪重變化

的速度分佈。

## 10. 加速度、phase 與 lap time

速度圖完成後，縱向加速度由離散運動學反推：

$$
a_{x,j}
\approx
\frac{v_{j+1}^2-v_j^2}{2(s_{j+1}-s_j)}
$$

內部點會平均前後 segment 的加速度。

phase 判斷：

$$
\text{phase}=
\begin{cases}
\text{accelerate}, & a_x>0.05 \\
\text{brake}, & a_x<-0.05 \\
\text{hold}, & \text{otherwise}
\end{cases}
$$

單圈時間使用梯形概念的 segment mean speed 積分：

$$
\Delta t_j
=
\frac{\Delta s_j}{\frac{1}{2}(v_j+v_{j+1})}
$$

$$
t_\text{lap}
=
\sum_j \Delta t_j
$$

## 11. 輸出資料

`SpeedPlan` 儲存主要結果：

- `lateral_speed_limit`：純側向與空力輪重限制後的速度上限
- `speed`：經前向加速 / 後向煞車 pass 後的最終速度
- `acceleration`：由速度分佈反推的縱向加速度
- `phase`：`accelerate` / `brake` / `hold`
- `fz_per_tire`：平均單輪垂直載重
- `wheel_loads`：四輪載重
- `theta`：簡化車身姿態
- `heave`：heave
- `downforce`：下壓力
- `drag`：阻力
- `cop`：空力中心
- `cl`、`cd`：空力係數
- `drive_system_accel_limit`：只看動力系統的最大加速度 trace
- `brake_system_decel_limit`：只看煞車系統的最大減速度 trace

`plot_speed_map()` 會輸出：

```text
Fast Iteration_Model/data/track_speed_map.png
```

`export_lap_log()` 會將每個離散點轉成 `Vehicle_state`，並輸出：

```text
Fast Iteration_Model/data/track_speed_map_lap_data.pkl
```

其中包含：

$$
x,y,s,v,a_x,a_y,F_x,F_y,F_z,\alpha,\kappa_\text{slip},M_x,M_z,RRT
$$

以及空力、COP、油門/煞車命令、馬達扭矩等欄位。

## 12. 重要假設與限制

目前理論最佳狀態的定義是「在目前賽道中心線與模型限制下，透過 quasi-static friction-limited speed planning 得到的最佳速度分佈」。它有幾個重要假設：

- 車輛路徑固定在賽道中心線，沒有最佳 racing line 搜尋。
- 每個點採用穩態輪重與空力姿態估算，沒有完整 transient suspension dynamics。
- 側向力依照輪重比例分配：

  $$
  F_{y,i}
  =
  \frac{F_{z,i}}{\sum_j F_{z,j}}
  F_{y,\text{total}}
  $$

- 輪胎合成力極限由查表與 convex hull 近似。
- 前向/後向 pass 是速度圖演算法，不是全域最佳控制求解器。
- 輪胎 slip angle / slip ratio 在輸出 lap log 時，是從 tire map 中找最接近目標 $F_x,F_y$ 的格點。

因此，這份結果適合用來做快速 iteration、比較車輛參數趨勢、估計理論速度圖與單圈時間；若要做更完整的賽車線最佳化或 transient lap simulation，需要再加入路徑最佳化與動態車輛模型。
