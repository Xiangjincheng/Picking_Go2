import cv2
import numpy as np

# 定义棋盘格尺寸
chessboard_size = (9, 6)
square_size = 0.025  # 棋盘格每个小格子的尺寸，单位为米

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
images = [...]  # 标定图像列表

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
