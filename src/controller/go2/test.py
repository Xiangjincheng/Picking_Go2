def sgbm(self, frame, x, y):
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(self.stereo_camera.cam_matrix_left, self.stereo_camera.distortion_l,
                                                                      self.stereo_camera.cam_matrix_right, self.stereo_camera.distortion_r, 
                                                                      self.stereo_camera.size, self.stereo_camera.R, self.stereo_camera.T)

    left_map1, left_map2 = cv2.initUndistortRectifyMap(self.stereo_camera.cam_matrix_left, self.stereo_camera.distortion_l, R1, P1, self.stereo_camera.size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(self.stereo_camera.cam_matrix_right, self.stereo_camera.distortion_r, R2, P2, self.stereo_camera.size, cv2.CV_16SC2)

    frame1 = frame[0:480, 0:640]
    frame2 = frame[0:480, 640:1280]

    imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)

    blockSize = 5
    numDisparities = 16 * 12  # 必须是16的倍数
    stereo = cv2.StereoSGBM_create(minDisparity=0,
                                   numDisparities=numDisparities,
                                   blockSize=blockSize,
                                   P1=8 * blockSize * blockSize,
                                   P2=32 * blockSize * blockSize,
                                   disp12MaxDiff=1,
                                   preFilterCap=31,
                                   uniquenessRatio=15,
                                   speckleWindowSize=50,
                                   speckleRange=32,
                                   mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    # 计算视差
    disparity = stereo.compute(img1_rectified, img2_rectified).astype(np.float32) / 16.0

    # 视差图优化
    disparity = cv2.medianBlur(disparity, 5)

    points_3d = cv2.reprojectImageTo3D(disparity, Q)

    rep_point = {'x': points_3d[y, x, 0], 'y': points_3d[y, x, 1], 'z': points_3d[y, x, 2]}
    print(rep_point)
    return rep_point