# Camera Calibration HW1

本作業主題為：實作相機校正，估測相機內參矩陣 (Intrinsic) 與每張影像的外參 (Extrinsic)。

## Core Constraints

- 禁止使用 `cv2.calibrateCamera` 或其他現成校正函式。
- 必須自行完成以下流程：
  1. 每張影像求單應性矩陣 $H_i$
  2. 由多張 $H_i$ 建立線性方程組求解 $B$
  3. 由 $B$ 做 Cholesky factorization 取得內參矩陣 $K$
  4. 利用 $K$ 與 $H_i$ 還原每張影像外參 $[R|t]$
  5. 代入助教提供的視覺化程式產生 3D 外參圖

## Folder Usage

- `data/`: 助教提供資料集 (預設讀取路徑)
- `my_data/`: 手機自行拍攝資料集
- `output/`: 你產生的輸出檔案（建議統一放這裡，方便助教批改）

## Environment Setup

### Windows (PowerShell)

```powershell

py -3.13 -m venv .env

.\.env\Scripts\Activate.ps1

python -m pip install --upgrade pip

python -m pip install -r requirement.txt
```

## Exec

```powershell
python camera_calibration.py
```

程式會：

- 讀取棋盤格影像
- 偵測角點
- 以手刻演算法估測內外參
- 顯示外參 3D 視覺化

## Show

`camera_calibration_show_extrinsics.py` 為助教提供的視覺化模組，主程式會直接呼叫其 `draw_camera_boards(...)` 來畫出外參結果。

若你只想檢視該檔內容：

```powershell
python camera_calibration_show_extrinsics.py
```

（此檔本身偏工具模組，通常由主程式呼叫。）

## Switch Dataset

目前主程式預設：

```python
images = glob.glob('data/*.jpg')
```

若要改成手機資料集，改為：

```python
images = glob.glob('my_data/*.jpg')
```

## Output Naming Suggestion

請將輸出放在 `output/`，並附簡短檔名說明，範例如下：

- `output/intrinsic_matrix.txt`: 相機內參矩陣 $K$
- `output/extrinsics_rvec_tvec.npy`: 每張影像的外參（旋轉向量 + 平移向量）
- `output/extrinsics_plot.png`: 外參 3D 視覺化截圖

建議在報告中新增一小段 `Output File Description`，逐一說明每個檔案用途，助教會更容易對照評分。
