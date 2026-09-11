#數據工具
import matplotlib.pyplot as plt
import os
import pandas as pd
from scipy.interpolate import interp1d
from pathlib import Path
import json
import numpy as np

# 取得目前這個 .py 檔案的絕對路徑
base_path = Path(__file__).resolve().parent

def file_search(filename):#尋找檔案
    print("Opening file...")
    if not os.path.exists(filename):
        print("File not found:",filename)
    else:
        print("Successfully found:",filename)

def datasetup(file_name): 
    file_search(file_name)
    df_raw = pd.read_excel(file_name, header=None)
    name_a = df_raw.iloc[0, 0]  # A1
    name_b = df_raw.iloc[0, 1]  # B1
    df = pd.read_excel(file_name)
    # 取出欄位
    a_array = df[name_a].values
    b_array = df[name_b].values
    lookup = interp1d(a_array, b_array, bounds_error=False, fill_value="extrapolate")
    return lookup

def dictionary_data_printer(data):#字典資料顯示器
    for key, value in data.items():
        print(f"{key}: {value}")
        
def data_graph(A_list,B_list,T = None,label = (None,None)):#畫圖工具
    x,y = label
    plt.figure(figsize=(6, 5))
    plt.plot(A_list, B_list, color='green')
    plt.title(T)
    plt.xlabel(x)
    plt.ylabel(y)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def save_data_excel(file_name,data):
    new_df = pd.DataFrame(data)
    # 將 DataFrame 匯出到 Excel 檔案
    try:
        new_df.to_excel(file_name, index=False) # index=False 不寫入 DataFrame 的索引
        print(f"\nSuccessfully exported data to '{file_name}'")
    except Exception as e:
        print(f"\nError exporting data to Excel: {e}")

def Total_data_counter(data,limit):
    data_quantity = len(data)
    if data_quantity > limit: #數據最低數量
        print("Enough data : ",data_quantity)
    else:
        print(f"Warning: Not enough data")

class DataExporter:
    def __init__(self, data):
        self.data = data

    def normalize_data(self):
        if isinstance(self.data, dict):
            return [{'wheel': k, **v} for k, v in self.data.items()]
        elif isinstance(self.data, list):
            return self.data
        else:
            raise TypeError("Unsupported data format")

    def recursive_convert(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: self.recursive_convert(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self.recursive_convert(v) for v in obj]
        else:
            return obj

    def to_json(self, filename):
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(self.recursive_convert(self.data), f, indent=2)
        print(f"Saved as JSON: {filename}")

    def to_excel(self, filename):
        rows = []
        for entry in self.normalize_data():
            row = {}
            for k, v in entry.items():
                if isinstance(v, dict):
                    row.update({f"{k}_{subk}": subv for subk, subv in v.items()})
                else:
                    row[k] = v
            rows.append(row)
        df = pd.DataFrame(rows)
        df.to_excel(filename, index=False)
        print(f"Saved as Excel: {filename}")
        
def json_to_excel(filename):
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)  # data 是 list of dict

    # 使用 pandas.json_normalize 展開巢狀結構
    df = pd.json_normalize(data, sep='.')

    # 存成 Excel
    df.to_excel("json_to_excel.xlsx", index=False)

    print("output excel")

