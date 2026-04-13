# Camera Calibration HW1

## intro

本研究旨在完成平面標定板之單眼相機校正，並以「自行實作」與「OpenCV 內建方法」進行系統性比較。研究問題聚焦於：在不直接呼叫內建校正解算器的前提下，能否僅依賴棋盤格角點對應，穩定估測相機內參矩陣與多視角外參，並以重投影誤差驗證其可用性。

方法上，本作業採用 Zhang calibration 的核心思想。首先由多張棋盤格影像建立平面世界座標與像素座標對應，接著以 DLT 估測各視角 homography，再由 homography 所形成之線性約束求得對偶絕對二次曲面相關矩陣，進而恢復內參矩陣 K。於此基礎上，利用 K 與各視角 homography 分解旋轉與平移，並以正交化步驟修正旋轉矩陣，最後轉換為 rvec 與 tvec 形式。

為評估結果的正確性與穩健性，本研究另以 OpenCV calibrateCamera 建立對照組，並輸出兩類指標：其一為像素域重投影誤差，其二為參數域差異（內參差異、每視角姿態差異）。此外，實驗流程支援替換資料集（data 與 my_data），以觀察不同拍攝條件下估測品質與方法一致性。

## implement

本節對實作流程進行技術性說明，對應程式主體可分為七個階段。

1. 特徵點偵測與資料建構
先建立棋盤格平面座標 objp（z=0），並逐張讀取影像，透過 findChessboardCorners 擷取角點。每張成功偵測之影像均累積一組 object points 與 image points，作為後續 homography 與參數估測輸入。

2. 單視角 homography 估測
對每張影像，利用世界平面點 (X, Y) 與影像點 (u, v) 建立 DLT 線性系統 A，透過 SVD 取得最小奇異值對應解，重塑為 3x3 矩陣 H，並以 H[2,2] 進行尺度正規化。

3. 內參線性解（Zhang constraints）
根據各視角 H 的列向量，構造 v_ij 向量並疊加成矩陣 V，求解 Vb=0 得到對稱矩陣 B 參數。接著由 B=K^{-T}K^{-1} 關係恢復內參。實作上先檢查 B 的正定性，必要時加入微小對角修正，再以 Cholesky 分解求得 K，並進行符號與尺度校正以確保 K[2,2]=1。

4. 外參恢復與旋轉正交化
已知 K 後，對每張 H 計算尺度因子 lambda，得到 r1、r2、t，並以 r3=r1×r2 補齊旋轉基底。由於數值誤差會破壞正交性，故使用 SVD 對近似旋轉矩陣做投影修正，使其落在 SO(3)，最後轉為 Rodrigues 向量 rvec，配對 tvec 作為每視角外參。

5. 重投影誤差評估
實作 mean_reprojection_error：以 projectPoints 將 3D 角點重投影至影像平面，計算與偵測角點之 L2 距離，並對所有點取平均，作為手刻流程在像素域的主要效能指標。

6. OpenCV 對照組與姿態差異分析
使用 calibrateCamera 取得 cv_mtx、cv_dist、cv_rvecs、cv_tvecs。接著計算兩種方法之重投影誤差差值、內參差值（fx, fy, cx, cy, skew），以及每張影像的姿態差異：旋轉差以相對旋轉矩陣角度表示，平移差以向量 L2 norm 表示。

7. 結果輸出與可視化
最終輸出 calibration_report.txt（整合誤差、內參、差異統計）、per_view_pose_diff.csv（逐視角差異），並產生兩張外參 3D 視覺化圖（手刻法與 OpenCV 各一張），以便進行質化檢查與報告撰寫。

## Setup

### Windows (PowerShell)

```powershell
py -3.13 -m venv .env
\.env\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirement.txt
```

## Exec

```powershell
python camera_calibration.py
```

## Switch Dataset

預設讀取：

```python
images = glob.glob('data/*.jpg')
```

如果要換成自己的資料，改成：

```python
images = glob.glob('my_data/*.jpg')
```

## Output detail

``` txt
└── /output
      ├── ./calibration_report.txt
      ├── ./per_view_pose_diff.csv
      ├── ./extrinsics_plot.png
      └── ./cv_extrinsics_plot.png
```

1. `calibration_report.txt`
內容：

- 手刻與 OpenCV 的誤差
- 兩組 intrinsic matrix
- OpenCV distortion coeffs
- 內參差異（abs / percent）
- 外參差異統計與 Top-3 差異視角

1. `per_view_pose_diff.csv`
內容：每張圖的 pose diff（rotation difference / translation difference）

2. `extrinsics_plot.png`
內容：手刻外參 3D

3. `cv_extrinsics_plot.png`
內容：OpenCV baseline 外參 3D
