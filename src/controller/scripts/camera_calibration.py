import cv2
import numpy as np

# 定义棋盘格尺寸
chessboard_size = (12, 8)
square_size = 0.033  # 棋盘格每个小格子的尺寸，单位为米

# 准备棋盘格点的世界坐标
obj_points = []
for i in range(chessboard_size[1]):
    for j in range(chessboard_size[0]):
        obj_points.append((j * square_size, i * square_size, 0))
obj_points = np.array(obj_points, dtype=np.float32)

# 存储角点的世界坐标和图像坐标
obj_points_list = []
img_points_list = []

# 读取标定图像
images = []  # 标定图像列表

for img_file in images:
    img = cv2.imread(img_file)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret:
        obj_points_list.append(obj_points)
        img_points_list.append(corners)
        
        # 可视化角点
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# 进行相机标定
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points_list, img_points_list, gray.shape[::-1], None, None)

# 打印标定结果
print("相机矩阵:\n", camera_matrix)
print("畸变系数:\n", dist_coeffs)


# Camera matrix (K):内参
#  [[1.74192606e+03 0.00000000e+00 2.94158188e+02]
#  [0.00000000e+00 1.51257698e+03 1.65678778e+02]
#  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
# Distortion coefficients:畸变
#  [[ 4.83697282e-03 -5.64171037e+01  2.06721865e-02  7.13074916e-03 2.35169075e+03]]
# 外参
# Rotation matrix (R):
#  [[ 0.96456357 -0.01655853  0.26333045]
#  [ 0.00657243  0.99922701  0.03875814]
#  [-0.26376868 -0.03565397  0.9639268 ]]
# Translation vector (t):
#  [[-163.14879239]
#  [ -49.43445986]
#  [1945.58037092]]

# w 橘子:40mm
# f = 1.51257698e+03
# W=roi.width