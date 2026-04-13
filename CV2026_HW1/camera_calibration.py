import numpy as np
import cv2
import glob
import os
import matplotlib.pyplot as plt
import camera_calibration_show_extrinsics as show
from PIL import Image
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# (8,6) is for the given testing images.
# If you use the another data (e.g. pictures you take by your smartphone), 
# you need to set the corresponding numbers.
corner_x = 7
corner_y = 7
objp = np.zeros((corner_x*corner_y,3), np.float32)
objp[:,:2] = np.mgrid[0:corner_x, 0:corner_y].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane.

# Make a list of calibration images
images = glob.glob('data/*.jpg')
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
# You need to comment these functions and write your calibration function from scratch.
# Notice that rvecs is rotation vector, not the rotation matrix, and tvecs is translation vector.
# In practice, you'll derive extrinsics matrixes directly. The shape must be [pts_num,3,4], and use them to plot.
def compute_homography(world_xy, image_uv):
    A = []
    for (X, Y), (u, v) in zip(world_xy, image_uv):
        A.append([-X, -Y, -1, 0, 0, 0, u * X, u * Y, u])
        A.append([0, 0, 0, -X, -Y, -1, v * X, v * Y, v])
    A = np.asarray(A, dtype=np.float64)
    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    return H / H[2, 2]


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


if len(objpoints) < 3:
    raise RuntimeError('Need at least 3 valid chessboard images for calibration.')

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

# Save calibration outputs for report and grading.
os.makedirs('output', exist_ok=True)
np.savetxt('output/intrinsic_matrix.txt', mtx, fmt='%.10f')
np.save('output/extrinsics_rvec_tvec.npy', extrinsics)
np.save('output/rvecs.npy', rvecs)
np.save('output/tvecs.npy', tvecs)

with open('output/calibration_summary.txt', 'w', encoding='utf-8') as f:
    f.write('Camera Calibration Summary\n')
    f.write(f'Valid images used: {len(objpoints)} / {len(images)}\n')
    f.write(f'Image size (w, h): {img_size}\n\n')
    f.write('Intrinsic matrix K:\n')
    np.savetxt(f, mtx, fmt='%.10f')

print('Saved: output/intrinsic_matrix.txt')
print('Saved: output/extrinsics_rvec_tvec.npy')
print('Saved: output/rvecs.npy')
print('Saved: output/tvecs.npy')
print('Saved: output/calibration_summary.txt')
"""
Write your code here




"""
# show the camera extrinsics
print('Show the camera extrinsics')
# plot setting
# You can modify it for better visualization
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
# camera setting
camera_matrix = mtx
cam_width = 0.064/0.1
cam_height = 0.032/0.1
scale_focal = 1600
# chess board setting
board_width = 8
board_height = 6
square_size = 1
# display
# True -> fix board, moving cameras
# False -> fix camera, moving boards
min_values, max_values = show.draw_camera_boards(ax, camera_matrix, cam_width, cam_height,
                                                scale_focal, extrinsics, board_width,
                                                board_height, square_size, True)

X_min = min_values[0]
X_max = max_values[0]
Y_min = min_values[1]
Y_max = max_values[1]
Z_min = min_values[2]
Z_max = max_values[2]
max_range = np.array([X_max-X_min, Y_max-Y_min, Z_max-Z_min]).max() / 2.0

mid_x = (X_max+X_min) * 0.5
mid_y = (Y_max+Y_min) * 0.5
mid_z = (Z_max+Z_min) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, 0)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

ax.set_xlabel('x')
ax.set_ylabel('z')
ax.set_zlabel('-y')
ax.set_title('Extrinsic Parameters Visualization')
plt.savefig('output/extrinsics_plot.png', dpi=200, bbox_inches='tight')
print('Saved: output/extrinsics_plot.png')
plt.show()

#animation for rotating plot
"""
for angle in range(0, 360):
    ax.view_init(30, angle)
    plt.draw()
    plt.pause(.001)
"""
