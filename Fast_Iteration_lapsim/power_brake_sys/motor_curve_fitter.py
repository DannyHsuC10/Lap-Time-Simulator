import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline
from sklearn.linear_model import LinearRegression
from pathlib import Path

# 取得目前這個 .py 檔案的絕對路徑
base_path = Path(__file__).resolve().parent

import data_tool as dt

SPLINE_END_RPM = 12000
QUAD_END_RPM = 15500
LINEAR_END_RPM = 18500
ZERO_TORQUE_START_RPM = 18300 # 扭矩強制為 0 的起始點，我直接try error 試出這個數值，其實這個做法有點笨，直接將小於0的扭力設定成0最好
#預設變數
quad_coefs = None
lin_model = None
spline_model = None
# 讀取 Excel
file_name = base_path / "excel"/ "motor_data_raw.xlsx"# 讀取檔案名稱
output_file_name =  base_path/ "excel" / "motor_speed_vs_torque.xlsx" # 新增：輸出檔案名稱
dt.file_search(file_name)
df = pd.read_excel(file_name)
speed = df['speed'].values
torque = df['torque'].values

def predict_torque(x_val, spline_end_param, quad_end_param, linear_end_param, zero_torque_start_param):
    if x_val <= spline_end_param and spline_model is not None:
        return spline_model(x_val)
    elif spline_end_param < x_val <= quad_end_param and quad_coefs is not None:
        return np.polyval(quad_coefs, x_val)
    elif quad_end_param < x_val <= linear_end_param and lin_model is not None:
        return lin_model.predict(np.array(x_val).reshape(-1, 1))[0]
    elif x_val > zero_torque_start_param:
        return 0.0
    else: # 如果在任何区间之外，或者模型未训练，返回0
        return 0.0

# Spline處理區間
mask_spline = speed <= SPLINE_END_RPM#判斷區間
speed_spline = speed[mask_spline]#設定參數
torque_spline = torque[mask_spline]
dt.Total_data_counter(speed_spline,3)#三次曲線Spline處理希望至少需要4個點數據
spline_model = CubicSpline(speed_spline, torque_spline)
# 二次曲線區間
mask_quad = (speed >= SPLINE_END_RPM) & (speed <= QUAD_END_RPM)#判斷區間
speed_quad = speed[mask_quad]#設定參數
torque_quad = torque[mask_quad]
dt.Total_data_counter(speed_quad,2)#二次曲線Spline處理至少需要3個點數據
quad_coefs = np.polyfit(speed_quad, torque_quad, deg=2)
# 線性區間
mask_linear = (speed >= QUAD_END_RPM) & (speed <= LINEAR_END_RPM)
speed_linear = speed[mask_linear]
torque_linear = torque[mask_linear]
dt.Total_data_counter(speed_linear,1)#兩個點才能構成直線
lin_model = LinearRegression().fit(speed_linear.reshape(-1, 1), torque_linear)
#產生扭力數據
speed_fine_plot = np.linspace(speed.min(), speed.max() + 2000, 500) # 產生速度
torque_predicted = np.array([predict_torque(s, SPLINE_END_RPM, QUAD_END_RPM, LINEAR_END_RPM, ZERO_TORQUE_START_RPM) for s in speed_fine_plot])#產生扭力
torque_predicted[speed_fine_plot > ZERO_TORQUE_START_RPM] = 0# 超過設定成0
# 匯出平滑數據到 Excel
output_data = {'speed': speed_fine_plot,'torque': torque_predicted}
dt.save_data_excel(output_file_name,output_data)
#畫圖數據圖
dt.data_graph(speed_fine_plot,torque_predicted,T = 'Motor Speed vs Torque (After Fiting)',label = ('Motor Speed (RPM)','Torque (N·m)'))