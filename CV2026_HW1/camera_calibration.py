import numpy as np
import cv2
import glob
import os
import matplotlib.pyplot as plt
import camera_calibration_show_extrinsics as show
from PIL import Image

corner_x = 10
corner_y = 7
objp = np.zeros((corner_x*corner_y,3), np.float32)
objp[:,:2] = np.mgrid[0:corner_x, 0:corner_y].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane.

# Make a list of calibration images
images = glob.glob('my_data/*.jpg')
if len(images) == 0:
    raise RuntimeError('No calibration images found in data/*.jpg')

# Step through the list and search for chessboard corners
print('Start finding chessboard corners...')
for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    plt.imshow(gray)

    #Find the chessboard corners
    print('find the chessboard corners of',fname)
    ret, corners = cv2.findChessboardCorners(gray, (corner_x,corner_y), None)

    # If found, add object points, image points
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (corner_x,corner_y), corners, ret)
        plt.imshow(img)


#######################################################################################################
#                                Homework 1 Camera Calibration                                        #
#               You need to implement camera calibration(02-camera p.76-80) here.                     #
#   DO NOT use the function directly, you need to write your own calibration function from scratch.   #
#                                          H I N T                                                    #
#                        1.Use the points in each images to find Hi                                   #
#                        2.Use Hi to find out the intrinsic matrix K                                  #
#                        3.Find out the extrensics matrix of each images.                             #
#######################################################################################################
print('Camera calibration...')
img_size = gray.shape[::-1]

# [TODO DONE] Compute homography H from world points (X, Y) to image points (u, v) using DLT.
def compute_homography(world_xy, image_uv):
    A = []
    for (X, Y), (u, v) in zip(world_xy, image_uv):
        A.append([-X, -Y, -1, 0, 0, 0, u * X, u * Y, u])
        A.append([0, 0, 0, -X, -Y, -1, v * X, v * Y, v])
    A = np.asarray(A, dtype=np.float64)
    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    return H / H[2, 2]

# [TODO DONE] Compute the vector v_ij from homography H for indices i, j.
def v_ij(H, i, j):
    h = H.T
    return np.array([
        h[i, 0] * h[j, 0],
        h[i, 0] * h[j, 1] + h[i, 1] * h[j, 0],
        h[i, 1] * h[j, 1],
        h[i, 2] * h[j, 0] + h[i, 0] * h[j, 2],
        h[i, 2] * h[j, 1] + h[i, 1] * h[j, 2],
        h[i, 2] * h[j, 2],
    ], dtype=np.float64)

# [TODO DONE] Recover the intrinsic matrix K from the matrix B using Cholesky decomposition.
def intrinsic_from_B(B):
    # B = K^-T K^-1; recover K by Cholesky on B.
    if np.linalg.det(B) < 0:
        B = -B
    eigvals = np.linalg.eigvalsh(B)
    if np.any(eigvals <= 0):
        B = B + np.eye(3) * (1e-9 - eigvals.min())
    L = np.linalg.cholesky(B)
    K = np.linalg.inv(L.T)
    K = K / K[2, 2]
    if K[0, 0] < 0:
        K[:, 0] *= -1
    if K[1, 1] < 0:
        K[:, 1] *= -1
    return K

# [TODO DONE] Compute average reprojection error over all calibration views.
def mean_reprojection_error(K, dist, rvecs, tvecs, objpoints, imgpoints):
    total_error = 0.0
    total_points = 0
    for i, obj_pts in enumerate(objpoints):
        proj_pts, _ = cv2.projectPoints(
            obj_pts,
            rvecs[i].reshape(3, 1),
            tvecs[i].reshape(3, 1),
            K,
            dist,
        )
        err = np.linalg.norm(
            imgpoints[i].reshape(-1, 2) - proj_pts.reshape(-1, 2),
            axis=1,
        )
        total_error += err.sum()
        total_points += len(err)
    if total_points == 0:
        return np.nan
    return total_error / total_points


def rotation_angle_diff_deg(rvec_a, rvec_b):
    R_a, _ = cv2.Rodrigues(rvec_a.reshape(3, 1))
    R_b, _ = cv2.Rodrigues(rvec_b.reshape(3, 1))
    R_delta = R_a @ R_b.T
    trace_val = np.trace(R_delta)
    cos_theta = np.clip((trace_val - 1.0) * 0.5, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))

if len(objpoints) < 3:
    raise RuntimeError('Need at least 3 valid chessboard images for calibration.')

# [TODO DONE] Compute the homographies, intrinsic matrix, and extrinsic parameters for each image.
homographies = []
for obj_pts, img_pts in zip(objpoints, imgpoints):
    world_xy = obj_pts[:, :2]
    image_uv = img_pts.reshape(-1, 2)
    homographies.append(compute_homography(world_xy, image_uv))

V = []
for H in homographies:
    V.append(v_ij(H, 0, 1))
    V.append(v_ij(H, 0, 0) - v_ij(H, 1, 1))
V = np.asarray(V, dtype=np.float64)

_, _, Vt = np.linalg.svd(V)
b = Vt[-1]
B = np.array([
    [b[0], b[1], b[3]],
    [b[1], b[2], b[4]],
    [b[3], b[4], b[5]],
], dtype=np.float64)

mtx = intrinsic_from_B(B)
K_inv = np.linalg.inv(mtx)

rvecs = []
tvecs = []
extrinsics = []
for H in homographies:
    h1 = H[:, 0]
    h2 = H[:, 1]
    h3 = H[:, 2]

    lam = 1.0 / np.linalg.norm(K_inv @ h1)
    r1 = lam * (K_inv @ h1)
    r2 = lam * (K_inv @ h2)
    r3 = np.cross(r1, r2)
    t = lam * (K_inv @ h3)

    R_approx = np.column_stack((r1, r2, r3))
    U, _, Vt = np.linalg.svd(R_approx)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R[:, -1] *= -1

    rvec, _ = cv2.Rodrigues(R)
    rvecs.append(rvec.reshape(3))
    tvecs.append(t.reshape(3))
    extrinsics.append(np.concatenate((rvec.reshape(3), t.reshape(3))))

ret = True
dist = np.zeros(5, dtype=np.float64)
rvecs = np.asarray(rvecs, dtype=np.float64)
tvecs = np.asarray(tvecs, dtype=np.float64)
extrinsics = np.asarray(extrinsics, dtype=np.float64)

custom_reproj_error = mean_reprojection_error(
    mtx,
    dist,
    rvecs,
    tvecs,
    objpoints,
    imgpoints,
)

# OpenCV baseline for comparison.
cv_ret, cv_mtx, cv_dist, cv_rvecs, cv_tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    img_size,
    None,
    None,
)
cv_rvecs = np.asarray([rv.reshape(3) for rv in cv_rvecs], dtype=np.float64)
cv_tvecs = np.asarray([tv.reshape(3) for tv in cv_tvecs], dtype=np.float64)
cv_extrinsics = np.asarray(
    [np.concatenate((rv, tv)) for rv, tv in zip(cv_rvecs, cv_tvecs)],
    dtype=np.float64,
)
cv_reproj_error = mean_reprojection_error(
    cv_mtx,
    cv_dist,
    cv_rvecs,
    cv_tvecs,
    objpoints,
    imgpoints,
)

# Per-view pose differences between from-scratch and OpenCV baselines.
rot_diff_deg = np.asarray(
    [rotation_angle_diff_deg(rv, cv_rv) for rv, cv_rv in zip(rvecs, cv_rvecs)],
    dtype=np.float64,
)
t_diff = np.linalg.norm(tvecs - cv_tvecs, axis=1)
pose_diff_table = np.column_stack((
    np.arange(len(rot_diff_deg), dtype=np.float64),
    rot_diff_deg,
    t_diff,
))
worst_idx = int(np.argmax(rot_diff_deg + t_diff))

intrinsic_names = ['fx', 'fy', 'cx', 'cy', 'skew']
custom_intrinsic_vals = np.array([mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2], mtx[0, 1]], dtype=np.float64)
cv_intrinsic_vals = np.array([cv_mtx[0, 0], cv_mtx[1, 1], cv_mtx[0, 2], cv_mtx[1, 2], cv_mtx[0, 1]], dtype=np.float64)
intrinsic_abs_diff = custom_intrinsic_vals - cv_intrinsic_vals
intrinsic_pct_diff = np.where(
    np.abs(cv_intrinsic_vals) > 1e-12,
    100.0 * intrinsic_abs_diff / cv_intrinsic_vals,
    np.nan,
)

# [TODO DONE] Save calibration outputs for report and grading.
os.makedirs('output', exist_ok=True)
np.savetxt(
    'output/per_view_pose_diff.csv',
    pose_diff_table,
    fmt=['%d', '%.8f', '%.8f'],
    delimiter=',',
    header='view_index,rotation_diff_deg,translation_diff_norm',
    comments='',
)

with open('output/calibration_report.txt', 'w', encoding='utf-8') as f:
    f.write('Camera Calibration Report\n')
    f.write('=========================\n\n')
    f.write(f'Valid images used: {len(objpoints)} / {len(images)}\n')
    f.write(f'Image size (w, h): {img_size}\n\n')

    f.write('Error summary\n')
    f.write(f'From-scratch mean reprojection error (pixel): {custom_reproj_error:.6f}\n')
    f.write(f'OpenCV mean reprojection error (pixel): {cv_reproj_error:.6f}\n')
    f.write(f'OpenCV calibrateCamera RMS: {cv_ret:.6f}\n')
    f.write(f'Mean reprojection error difference (scratch - OpenCV): {custom_reproj_error - cv_reproj_error:.6f}\n\n')

    f.write('From-scratch intrinsic matrix K\n')
    np.savetxt(f, mtx, fmt='%.10f')
    f.write('\nOpenCV intrinsic matrix K\n')
    np.savetxt(f, cv_mtx, fmt='%.10f')
    f.write('\nOpenCV distortion coefficients\n')
    np.savetxt(f, cv_dist.reshape(1, -1), fmt='%.10f')
    f.write(f'\nFrobenius norm of K difference: {np.linalg.norm(mtx - cv_mtx):.6f}\n\n')

    f.write('Intrinsic parameter comparison\n')
    f.write('name,from_scratch,opencv,abs_diff,pct_diff\n')
    for i, name in enumerate(intrinsic_names):
        pct_val = intrinsic_pct_diff[i]
        pct_str = 'nan' if np.isnan(pct_val) else f'{pct_val:.6f}'
        f.write(
            f'{name},{custom_intrinsic_vals[i]:.8f},{cv_intrinsic_vals[i]:.8f},'
            f'{intrinsic_abs_diff[i]:.8f},{pct_str}\n'
        )
    f.write('\n')

    combined = rot_diff_deg + t_diff
    top_k = np.argsort(-combined)[:3]
    f.write('Pose difference statistics\n')
    f.write(f'Rotation diff mean (deg): {rot_diff_deg.mean():.6f}\n')
    f.write(f'Rotation diff max (deg): {rot_diff_deg.max():.6f}\n')
    f.write(f'Translation diff mean (L2 norm): {t_diff.mean():.6f}\n')
    f.write(f'Translation diff max (L2 norm): {t_diff.max():.6f}\n')
    f.write(f'Largest combined difference view index: {worst_idx}\n\n')

    f.write('Top 3 views by combined pose difference\n')
    f.write('view_index,rotation_diff_deg,translation_diff_norm,combined_score\n')
    for idx in top_k:
        f.write(
            f'{idx},{rot_diff_deg[idx]:.8f},{t_diff[idx]:.8f},{combined[idx]:.8f}\n'
        )

print('Saved: output/per_view_pose_diff.csv')
print('Saved: output/calibration_report.txt')

def save_extrinsics_plot(camera_matrix, extrinsics_data, output_path, title):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    cam_width = 0.064 / 0.1
    cam_height = 0.032 / 0.1
    scale_focal = 1600
    board_width = 8
    board_height = 6
    square_size = 1

    min_values, max_values = show.draw_camera_boards(
        ax,
        camera_matrix,
        cam_width,
        cam_height,
        scale_focal,
        extrinsics_data,
        board_width,
        board_height,
        square_size,
        True,
    )

    X_min = min_values[0]
    X_max = max_values[0]
    Y_min = min_values[1]
    Y_max = max_values[1]
    Z_min = min_values[2]
    Z_max = max_values[2]
    max_range = np.array([X_max - X_min, Y_max - Y_min, Z_max - Z_min]).max() / 2.0

    mid_x = (X_max + X_min) * 0.5
    mid_y = (Y_max + Y_min) * 0.5
    mid_z = (Z_max + Z_min) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, 0)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlabel('x')
    ax.set_ylabel('z')
    ax.set_zlabel('-y')
    ax.set_title(title)
    fig.savefig(output_path, dpi=200, bbox_inches='tight')
    print(f'Saved: {output_path}')


# show and save extrinsics plots
print('Show the camera extrinsics')
save_extrinsics_plot(
    mtx,
    extrinsics,
    'output/extrinsics_plot.png',
    'Extrinsic Parameters Visualization (From-scratch)',
)
save_extrinsics_plot(
    cv_mtx,
    cv_extrinsics,
    'output/cv_extrinsics_plot.png',
    'Extrinsic Parameters Visualization (OpenCV)',
)
plt.show()

