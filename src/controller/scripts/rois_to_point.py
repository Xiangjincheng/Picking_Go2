import rospy
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from interfaces_msgs.msg import Rois

class RoisToPoint:
    def __init__(self):
        rospy.init_node('rois_to_point', anonymous=True)
        rospy.loginfo("节点:rois_to_point, 已启动!")
        self.bridge=CvBridge()

        # rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.intrinsics_callback)      
        rospy.Subscriber('rois', Rois, self.rois_callback)     

        self.publisher_obj_pos = rospy.Publisher('obj_pos', Point, queue_size = 10)

    def rois_callback(self, rois_msg):
        for roi in rois_msg.rois:
            mid_x=roi.x_offset +roi.width*0.5
            mid_y=roi.y_offset +roi.height *0.5
            mid_x=int(mid_x)
            mid_y=int(mid_y)

            w = 40
            f = 1.51257698e+03
            W = roi.width
            R = np.array([[ 0.96456357, -0.01655853, 0.26333045],
                        [ 0.00657243, 0.99922701, 0.03875814],
                        [-0.26376868, -0.03565397, 0.9639268 ]], dtype=np.float32)
            t = np.array([[-163.14879239],
                        [ -49.43445986],
                        [1945.58037092]], dtype=np.float32)

            depth = w*f/W
            piex_vector = np.array([[mid_x],
                                    [mid_y],
                                    [depth]], dtype=np.float32)
            point_vertor = R.dot(piex_vector) + t
            msg = Point()
            msg.x, msg.y, msg.z = point_vertor
            self.publisher_obj_pos.publish(msg) 

    def depth_image_callback(self, depth_image):
        self.depth_iamge = depth_image

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = RoisToPoint()
    node.run()
