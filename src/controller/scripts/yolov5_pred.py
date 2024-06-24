import rospy
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from interfaces_msgs.msg import Rois, Targets

import numpy as np
import cv2
import pyrealsense2 as rs 
from cv_bridge import CvBridge
import torch 
from numpy import random

import sys
sys.path.append("/home/cheng/unitree_noetic/src/controller/yolov5_pak")
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
        self.img_size = 640
        self.device = select_device('cpu')
        self.half = self.device.type != 'cpu'
        weights = '/home/cheng/unitree_noetic/src/controller/yolov5_pak/best_3_20.pt'
        self.model = attempt_load(weights , device=self.device, inplace=True, fuse=True)  # load model  

        rospy.Subscriber('align_image', Image, self.rs_image_callback)
        self.publisher_pred_image = rospy.Publisher('pred_image', Image, queue_size = 10)
        self.publisher_obj_pos = rospy.Publisher('obj_pos', Point, queue_size = 10)
        #self.publisher_rois_result = rospy.Publisher('rois', Rois, queue_size = 10)


    def rs_image_callback(self, align_image_msg):
        align_image = self.bridge.imgmsg_to_cv2(align_image_msg, 'bgr8')
        height, width, _ = align_image.shape
        color_image = height[:, :width // 2, :]
        depth_colormap = height[:, width // 2:, :]
        depth_image = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2GRAY)
        depth_image = cv2.convertScaleAbs(depth_image, alpha=1.0 / 0.03)

        #color_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
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
            obj_pos_msg = self.pixel_to_point(depth_image, pred[0])
            #rois_msg = self.pred_to_rois(pred[0])

        pred_image_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        self.publisher_pred_image.publish(pred_image_msg)     # publish predicted image

        self.publisher_obj_pos.publish(obj_pos_msg)
        #self.publisher_rois_result.publish(rois_msg)        #publish rois

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
    
    def pixel_to_point(self,depth_image, pred):
        obj_pos_msg = Targets()
        if pred !=None:
            for target in pred:
                obj_pos = Point()
                mid_x=int((target[0].item() + target[2].item()) / 2)
                mid_y=int((target[1].item() + target[3].item()) / 2)
                if depth_image[mid_y][mid_x] != 0:
                    camera_coordinate = rs.rs2_deproject_pixel_to_point(rs.intrinsics(), [mid_x,mid_y], depth_image[mid_y][mid_x]/1000)
                    obj_pos.x, obj_pos.y, obj_pos.z = camera_coordinate[0], camera_coordinate[1], camera_coordinate[2]
                    obj_pos_msg.targets.append(obj_pos)
        return obj_pos_msg               

    #predicted result to rois
    # def pred_to_rois(self, pred):
    #     rois_msg = Rois()
    #     if pred !=None:
    #         for target in pred:
    #             roi = RegionOfInterest()
    #             roi.x_offset = int(target[0].item())
    #             roi.y_offset = int(target[1].item())
    #             roi.width = int(target[2].item()-target[0].item())
    #             roi.height = int(target[3].item()-target[1].item())
    #             rois_msg.rois.append(roi)
    #     return rois_msg


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Yolov5Pred()
    node.run()
