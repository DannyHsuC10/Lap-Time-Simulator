---
layout: base
---

# MATLAB Magic Formula API 說明文件

## 📌 模型定位

本模組為 **Magic Formula 6.1.2 (MF6.1.2)** 的 MATLAB 實現版本，提供統一 API 介面：[MF API](../MF_api.m)

```
MF_api(cmd, FZ, SA, SL, IA, P, tir)
```
此 MATLAB API 設計重點為：

> 將複雜的 MF6.1.2 公式封裝成單一簡潔接口

讓使用者只需：

```
MF_api("FX", ...)
```

即可取得結果，而不需理解內部所有公式細節。計算結果與python版本幾乎相同，所以這部分主要介紹使用方法。

## Magic formula 公式

### 綜合工況

1. [Fx](../MF612_Fx.m)
1. [Fy](../MF612_Fy.m)
1. [Mx](../MF612_Mx.m)
1. [Mz](../MF612_Mz.m)

### 純工況

因為內容重複通常不會直接使用，所以api沒有直接進行連接。

1. [Fx0](../Base_func/MF612_Fx0.m)
1. [Fy0](../Base_func/MF612_Fy0.m)
1. [Mz0](../Base_func/MF612_Mz0.m)

## 🎯 核心 API 介面

### 函式原型

```matlab
function out = MF_api(cmd, FZ, SA, SL, IA, P, tir)
```

---

### 📥 輸入參數

| 參數        | 單位     | 說明                    |
| --------- | ------ | --------------------- |
| `cmd`     | string | 指令："Fx" / "Fy" / "Mz" |
| `FZ`      | N      | 法向力                   |
| `SA`      | deg    | 側偏角                   |
| `SL`      | -      | 縱滑比                   |
| `IA`      | deg    | 外傾角                   |
| `P`       | Pa     | 胎壓                    |
| `mf_file` | string | TIR 檔案路徑              |

---

### 📤 輸出

| cmd    | 回傳   |
| ------ | ---- |
| `"FX"` | 縱向力  |
| `"FY"` | 橫向力  |
| `"MZ"` | 回正力矩 |
| `"MX"` | 翻轉力矩 |

##  CalculationInput 功能

```matlab
function MF = CalculationInput(FZ,SA,SL,IA,P,tir)
```

功能：

1. 讀取 `.tir` 檔(這部分將透過另外一個tir載入工具避免重複載入tir效率降低)
2. 封裝輸入工況
3. 建立 MF 計算所需完整 struct

---

###  自動設定的關鍵物理參數

| 參數       | 來源              |
| -------- | --------------- |
| `MF.PIO` | NOMPRES         |
| `MF.FZO` | FNOMIN          |
| `MF.Ro`  | UNLOADED_RADIUS |
| `MF.V`   | LONGVL          |

---

### Lambda Scaling Factors

本模型保留完整 λ scaling 架構：

例如：

```matlab
MF.lambdaMuyMi = 1;
MF.lambdaKya = 1;
MF.lambdaMuxMi = 1;
```

用途：

* 模型調整
* 模擬抓地力降低 (例如改為 0.6)
* 進行 sensitivity analysis
* 實車校正

---

#### 常用可調整參數

| 參數            | 意義                  |
| ------------- | ------------------- |
| `lambdaMuyMi` | 橫向峰值摩擦係數            |
| `lambdaMuxMi` | 縱向峰值摩擦係數            |
| `lambdaKya`   | Cornering stiffness |
| `lambdaCy`    | Shape factor        |
| `lambdaEy`    | Curvature factor    |
| `epsilonX/Y`  | 數值穩定項               |

---

## 📂 TIR 檔讀取流程(用於輔助整個輪胎模型運作)
> **使用 load_tir 讀取檔案後放入api中tir的欄位，使用方法如下。**

```matlab
tir = load_tir("MF612.tir");
```

### load_tir()

功能：

* 逐行解析 TIR 檔案
* 自動忽略：

  * 註解 `$`
  * Section header `[SECTION]`
* 將 key-value 轉成 struct

---

### parse_value()

功能：

* 嘗試將字串轉為數值
* 若失敗則保留字串

## 📊 使用範例

```matlab
Fx = MF_api("FX", 800, 0.0, 0.1, 0.0, 80000, tir);

Fy = MF_api("FY", 800, 3.0, 0.0, 0.0, 80000, tir);

Mz = MF_api("MZ", 800, 3.0, 0.0, 0.0, 80000, tir);
```
