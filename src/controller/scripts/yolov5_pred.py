import rospy
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import Image
from interfaces_msgs.msg import Rois

import numpy as np
import cv2
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

        rospy.Subscriber('rs_image', Image, self.rs_image_callback)
        self.publisher_pre_result = rospy.Publisher('pred_image', Image, queue_size = 10)
        self.publisher_rois_result = rospy.Publisher('rois', Rois, queue_size = 10)


    def rs_image_callback(self, image):
        # imgsz = check_img_size(self.img_size, s=self.model.stride.max())
        
        # color_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        # mask = np.zeros([color_image.shape[0], color_image.shape[1]], dtype=np.uint8)
        # mask[0:480, 320:640] = 255
        # imgs = [None]
        # imgs[0] = color_image
        # im0s = imgs.copy()
        # img = [letterbox(x, new_shape=imgsz)[0] for x in im0s]
        # img = np.stack(img, 0)
        # img = img[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR to RGB, to 3x416x416, uint8 to float32
        # img = np.ascontiguousarray(img, dtype=np.float16 if self.half else np.float32)
        # img /= 255.0  # 0 - 255 to 0.0 - 1.0
        
        #########################待测试############################
        color_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
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
            rois_msg = self.pred_to_rois(pred[0])

        pred_image_msg = self.bridge.cv2_to_imgmsg(result_img, encoding="bgr8")
        self.publisher_pre_result.publish(pred_image_msg)     # publish predicted image
        self.publisher_rois_result.publish(rois_msg)        #publish rois

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
    
    #predicted result to rois
    def pred_to_rois(self, pred):
        rois_msg = Rois()
        if pred !=None:
            for target in pred:
                roi = RegionOfInterest()
                roi.x_offset = int(target[0].item())
                roi.y_offset = int(target[1].item())
                roi.width = int(target[2].item()-target[0].item())
                roi.height = int(target[3].item()-target[1].item())
                rois_msg.rois.append(roi)
        return rois_msg


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Yolov5Pred()
    node.run()
