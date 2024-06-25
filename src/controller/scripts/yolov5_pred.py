import rospy
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from interfaces_msgs.msg import Rois, Targets, Images

import numpy as np
import cv2
import pyrealsense2 as rs 
from cv_bridge import CvBridge
import torch 
from numpy import random

import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
yolov5_pak_dir = os.path.join(current_dir, "..", "yolov5_pak")
sys.path.append(yolov5_pak_dir)

from models.experimental import attempt_load
from utils.dataloaders import LoadStreams, LoadImages
from utils.torch_utils import select_device
from utils.augmentations import letterbox
from utils.general import check_img_size, make_divisible, non_max_suppression


 
class Yolov5Pred:
    def __init__(self):
        rospy.init_node('yolov5_pred', anonymous=True)
        rospy.loginfo("节点:yolov5_pred, 已启动!")

        self.bridge=CvBridge()
        self.intrinsics = rs.intrinsics()
        self.depth_iamge = Image()
        self.img_size = 640
        self.device = select_device('cpu')
        self.half = self.device.type != 'cpu'
        weights = os.path.join(current_dir, "..", "yolov5_pak", "best_3_20.pt")
        self.model = attempt_load(weights , device=self.device, inplace=True, fuse=True)  # load model  

        rospy.Subscriber('/camera/color/image_raw', Image, self.rs_image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.intrinsics_callback)

        self.publisher_pred_image = rospy.Publisher('pred_image', Image, queue_size = 10)
        self.publisher_obj_pos = rospy.Publisher('obj_pos', Targets, queue_size = 10)


    def rs_image_callback(self, color_image_msg):
        color_image = self.bridge.imgmsg_to_cv2(color_image_msg, 'bgr8')
        imgsz = check_img_size(self.img_size, s=self.model.stride.max())
        # Create mask
        mask = np.zeros_like(color_image[:, :, 0], dtype=np.uint8)
        mask[0:480, 320:640] = 255
   
        # Letterbox and preprocess image
        img = [letterbox(color_image, new_shape=imgsz)[0]]
        img = np.stack(img, 0)
        img = img[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR to RGB, to 3x416x416, uint8 to float32
        img = np.ascontiguousarray(img, dtype=np.float16 if self.half else np.float32) / 255.0
        # Ensure contiguous memory for performance
        img = np.ascontiguousarray(img)
        # Get detections
        img = torch.from_numpy(img).to(self.device)
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        pred = self.model(img)[0]
        pred = non_max_suppression(pred, 0.25, 0.45)

        if pred[0] is not None:
            result_img = self.draw_boxes(color_image, pred[0])
            obj_pos_msg = self.pixel_to_point(pred[0])

        pred_image_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        self.publisher_pred_image.publish(pred_image_msg)     # publish predicted image

        self.publisher_obj_pos.publish(obj_pos_msg)

    def depth_image_callback(self, depth_image):
        self.depth_iamge = depth_image

    def intrinsics_callback(self, msg):
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.K[2]
        self.intrinsics.ppy = msg.K[5]
        self.intrinsics.fx = msg.K[0]
        self.intrinsics.fy = msg.K[4]
        if msg.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intrinsics.model = rs.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in msg.D]

    #draw target box
    def draw_boxes(self, img, pred):
        for det in pred:
            if det is not None and det.numel() == 6:
                det = det.cpu().numpy()
                x1, y1, x2, y2, conf, cls = det
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{int(cls)}: {conf:.2f}'
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return img
    
    def pixel_to_point(self, pred):
        obj_pos_msg = Targets()
        depth_image = self.bridge.imgmsg_to_cv2(self.depth_iamge)
        if pred !=None:
            for target in pred:
                obj_pos = Point()
                mid_x=int((target[0].item() + target[2].item()) / 2.0)
                mid_y=int((target[1].item() + target[3].item()) / 2.0)
                #rospy.loginfo(f"Depth value at pixel ({mid_x}, {mid_y}): {depth_image[mid_y][mid_x]} mm")
                if depth_image[mid_y][mid_x] != 0:
                    camera_coordinate = rs.rs2_deproject_pixel_to_point(self.intrinsics, [mid_x,mid_y], depth_image[mid_y][mid_x]/1000)
                    obj_pos.x, obj_pos.y, obj_pos.z = camera_coordinate[0], camera_coordinate[1], camera_coordinate[2]
                    obj_pos_msg.targets.append(obj_pos)

        return obj_pos_msg               

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Yolov5Pred()
    node.run()
