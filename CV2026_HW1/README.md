# Camera Calibration HW1

## Introduction

This study aims to perform monocular camera calibration using a planar calibration board and to compare a hand-crafted implementation with OpenCV’s built-in calibration function. The main research question is whether it is possible to stably estimate the camera intrinsic matrix and multi-view extrinsic parameters solely from chessboard corner correspondences, without directly calling the built-in calibration solver, and to validate the usability of the results through reprojection error.

In terms of methodology, this assignment adopts Zhang’s calibration method. First, correspondences between planar world coordinates and image pixel coordinates are established from multiple chessboard images. Next, the homography of each view is estimated using DLT. The linear constraints induced by these homographies are then used to solve for the matrix related to the dual image of the absolute conic, from which the intrinsic matrix \(K\) is recovered. Based on \(K\) and the homography of each view, the rotation and translation components are decomposed, and an orthogonalization step is applied to refine the rotation matrix. Finally, the results are converted into the forms of rvec and tvec.

To evaluate the correctness and robustness of the results, this study also uses OpenCV’s `calibrateCamera` as a baseline and reports two types of metrics:

1. Reprojection error in the pixel domain  
2. Parameter-domain differences, including intrinsic parameter differences and per-view pose differences

## Implement

This section describes the technical workflow of the implementation, which can be divided into seven stages.

1. **Feature Detection and Data Construction**  
   First, the planar chessboard coordinates `objp` are created with \(z=0\). Each image is then loaded one by one, and chessboard corners are detected using `findChessboardCorners`. For every successfully detected image, one set of object points and image points is collected as input for subsequent homography and parameter estimation.

2. **Single-View Homography Estimation**  
   For each image, a DLT linear system \(A\) is constructed from the world planar points \((X, Y)\) and image points \((u, v)\). SVD is then used to obtain the solution corresponding to the smallest singular value, which is reshaped into a \(3 \times 3\) matrix \(H\). Finally, scale normalization is performed using `H[2,2]`.

3. **Linear Intrinsic Solution (Zhang Constraints)**  
   Based on the column vectors of each homography \(H\), the vectors \(v_{ij}\) are constructed and stacked into a matrix \(V\). Solving \(Vb=0\) yields the parameters of the symmetric matrix \(B\). The intrinsic matrix is then recovered from the relationship \(B = K^{-T}K^{-1}\). In practice, the positive-definiteness of \(B\) is checked first. If necessary, a small diagonal perturbation is added, and then Cholesky decomposition is applied to recover \(K\). Sign and scale corrections are further performed to ensure that `K[2,2] = 1`.

4. **Extrinsic Recovery and Rotation Orthogonalization**  
   Once \(K\) is known, a scale factor \(\lambda\) is computed for each homography \(H\), and then \(r_1\), \(r_2\), and \(t\) are obtained. The third rotation basis vector is completed by \(r_3 = r_1 \times r_2\). Since numerical errors may break orthogonality, SVD is used to project the approximate rotation matrix onto \(SO(3)\). The final rotation matrix is then converted into a Rodrigues vector `rvec` and paired with `tvec` as the extrinsic parameters for each view.

5. **Reprojection Error Evaluation**  
   The function `mean_reprojection_error` is implemented to project 3D corner points back onto the image plane using `projectPoints`, compute the L2 distance between projected points and detected corners, and average the results over all points. This serves as the main pixel-domain performance metric for the hand-crafted pipeline.

6. **OpenCV Baseline and Pose Difference Analysis**  
   OpenCV’s `calibrateCamera` is used to obtain `cv_mtx`, `cv_dist`, `cv_rvecs`, and `cv_tvecs`. The reprojection error difference between the two methods is then computed, along with intrinsic parameter differences (`fx`, `fy`, `cx`, `cy`, and `skew`) and per-image pose differences. Rotation difference is measured by the angle of the relative rotation matrix, while translation difference is measured by the L2 norm of the translation vector difference.

7. **Result Output**  
   The final outputs include `calibration_report.txt` (containing integrated error analysis, intrinsic parameters, and difference statistics), `per_view_pose_diff.csv` (containing per-view differences), and two 3D visualizations of extrinsic parameters (one for the hand-crafted implementation and one for OpenCV).

## Setup

### Windows (PowerShell)

> Note: You may choose any Python environment manager you prefer, such as `venv` or `conda`. The example below uses `venv`.

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

```python
images = glob.glob('[self_data]/*.jpg')
```

## Output detail

``` txt
└── /output
      ├── ./calibration_report.txt
      ├── ./per_view_pose_diff.csv
      ├── ./extrinsics_plot.png
      └── ./cv_extrinsics_plot.png
```

- `calibration_report.txt`
      Contains the errors of the hand-crafted implementation and OpenCV, the two intrinsic matrices, OpenCV distortion coefficients, intrinsic parameter differences (absolute / percentage), extrinsic difference statistics, and the top 3 views with the largest discrepancies.

- `per_view_pose_diff.csv`
      Contains the pose difference of each image, including rotation difference and translation difference.
- `extrinsics_plot.png`
      3D visualization of the extrinsic parameters from the hand-crafted implementation.

- `cv_extrinsics_plot.png`
      3D visualization of the extrinsic parameters from the OpenCV baseline.
