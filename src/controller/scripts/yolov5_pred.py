import rospy
import numpy as np
import cv2
import torch 
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
yolov5_pak_dir = os.path.join(current_dir, "..", "yolov5_pak")
sys.path.append(yolov5_pak_dir)

from models.experimental import attempt_load
from utils.torch_utils import select_device
from utils.augmentations import letterbox
from utils.general import check_img_size, non_max_suppression

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, RegionOfInterest
from interfaces.msg import Rois

class Yolov5Pred:
    def __init__(self):
        rospy.init_node('yolov5_pred', anonymous=True)
        rospy.loginfo("节点:yolov5_pred, 已启动!")

        self.img_size = (480, 640)
        self.color_img = Image()
        self.bridge=CvBridge()
        self.device = select_device('cpu')
        self.half = self.device.type != 'cpu'
        weights = os.path.join(current_dir, "..", "yolov5_pak", "best_3_20.pt")
        self.model = attempt_load(weights , device=self.device, inplace=True, fuse=True)  # load model  

        #rospy.Subscriber('camera_image', Image, self.rs_image_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.rs_image_callback)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        
        self.publisher_pred_image = rospy.Publisher('pred_image', Image, queue_size = 10)
        self.publisher_rois = rospy.Publisher('region_of_interest', Rois, queue_size = 10)

    def timer_callback(self, event):
        color_image = self.bridge.imgmsg_to_cv2(self.color_img, 'bgr8')
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
        pred = non_max_suppression(pred, 0.65, 0.45)[0]

        if len(pred) != 0:
            result_img = self.draw_boxes(color_image, pred)
            roi_msg = self.pred_to_roi(pred)
        else:
            result_img = color_image
            roi_msg = Rois()

        pred_image_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        self.publisher_pred_image.publish(pred_image_msg)     # publish predicted image
        self.publisher_rois.publish(roi_msg)        #publish rois
        
    def rs_image_callback(self, color_image_msg): 
    	self.color_img = color_image_msg

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

    def pred_to_roi(self, pred):
        msg=Rois()
        for e in pred:
            m = RegionOfInterest()
            m.x_offset = int(e[0].item())
            m.y_offset = int(e[1].item())
            m.width = int(e[2].item()-e[0].item())
            m.height = int(e[3].item()-e[1].item())
            msg.rois.append(m)
            
        msg.rois.sort(key=lambda roi: roi.x_offset)
        return msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Yolov5Pred()
    node.run()
