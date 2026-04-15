# Two-View SfM HW2

## Introduction

This homework implements a two-view Structure from Motion (SfM) pipeline to reconstruct sparse 3D points from an image pair. The goal is to estimate camera geometry from feature correspondences, recover relative camera pose, triangulate 3D points, and export a textured `.obj` model for visualization in Blender.

Supported datasets:

1. `Statue` (`Statue1.bmp`, `Statue2.bmp`, `Statue_calib.txt`)
2. `Mesona` (`Mesona1.JPG`, `Mesona2.JPG`, `Mesona_calib.txt`)
3. `Matcha` (`my_data/Matcha1.jpg`, `my_data/Matcha2.jpg`, `my_data/Matcha_calib.txt`)
4. `Medicine` (`my_data/Medicine1.jpg`, `my_data/Medicine2.jpg`, `my_data/Medicine_calib.txt`)

The final outputs include recovered camera matrices, sparse 3D points, visibility-filtered mesh data, and model files that can be directly loaded into Blender.

## Implement

This section describes the technical workflow of the implementation, which can be divided into six stages.

1. **Feature Detection and Matching**  
 Keypoints and descriptors are extracted from both views using SIFT/SURF/ORB with automatic fallback, followed by ratio-test matching and geometric outlier removal to improve correspondence quality.

2. **Fundamental Matrix Estimation**  
 The normalized 8-point algorithm is used to estimate the fundamental matrix, and RANSAC is applied to robustly select inliers under noise and mismatches.

3. **Essential Matrix and Camera Pose Recovery**  
 Using camera intrinsics from calibration files, the essential matrix is computed by `E = K2' * F * K1`. The matrix is decomposed into four pose candidates, and the physically valid one is selected via cheirality (positive depth) constraints.

4. **Triangulation and Point Filtering**  
 Sparse 3D points are reconstructed using DLT triangulation. Reprojection error is then evaluated to remove unstable points and retain cleaner geometry.

5. **Visibility Checking and Mesh Construction**  
 The visibility logic in `CheckVisible.m` is used to validate view consistency, and `obj_main.m` organizes valid vertices/faces for mesh export.

6. **Result Export**  
 The pipeline saves intermediate SfM results (`sfm_results.mat`) and exports `.obj/.mtl` assets with texture image references for Blender-based inspection.

Main scripts:

- `sfm_main.m`
- `generate_calib.m`

TA:

- `obj_main.m`
- `CheckVisible.m`

## Setup

1. Install MATLAB (Install Computer Vision Toolbox in MATLAB).
2. (Optional) Install Blender for model visualization.
3. Open this folder in VS Code.
4. Ensure the `data/` and `imgs/` folders contain the required inputs.
5. For own photos (`Matcha`, `Medicine`), keep images and calibration in `my_data/`.

> Note: Must install [MATLAB](https://www.mathworks.com/downloads/) and [Blender](https://www.blender.org/).

## Exec

> Note:
>
> - The `matlab` command must be available in your system `PATH` (set this manually).
> - If `matlab` is not in `PATH`, run MATLAB with its full executable path.

```powershell
matlab -batch "results = sfm_main('Statue');"
"C:\Program Files\MATLAB\R2024b\bin\matlab.exe" -batch "results = sfm_main('Statue');"
```

## Switch Dataset

```matlab
results = sfm_main('Mesona');
results = sfm_main('Statue');
results = sfm_main('Matcha');
results = sfm_main('Medicine');
```

## Generate calibration txt for new images

`generate_calib` now uses one simple call and chooses the best method automatically:

1. Try checkerboard calibration first (if checkerboard images are available).
2. Fallback to FOV approximation when checkerboard data is unavailable.

Example:

```matlab
Klist = generate_calib('Matcha');
```

The file is written to the dataset folder by default (`my_data/` or `data/`) with
`Camera A/B/...` and `K1/K2/...` blocks. Current `sfm_main` uses `K1` and `K2`
for two-view reconstruction.

## Output detail

```txt
└── /output
  ├── /Mesona
  │     ├── model1.obj
  │     ├── model1.mtl
  │     └── sfm_results.mat
  └── /Statue
        ├── model1.obj
        ├── model1.mtl
        └── sfm_results.mat
```

- `model1.obj`  
  Exported 3D geometry that can be loaded in Blender.

- `model1.mtl`  
  Material file associated with the exported `.obj` model.

- `sfm_results.mat`  
  MATLAB data file containing intermediate and final SfM reconstruction results.

## How to use Blender

See [Blender.md](./guide/Blender.md).
