import rospy
import pyrealsense2 as rs 
from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from interfaces_msgs.msg import Rois

class RoisToPoint:
    def __init__(self):
        rospy.init_node('rois_to_point', anonymous=True)
        rospy.loginfo("节点:rois_to_point, 已启动!")
        self.bridge=CvBridge()
        self.intrinsics = rs.intrinsics()
        
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.intrinsics_callback)      
        rospy.Subscriber('rois', Rois, self.rois_callback)     

        self.publisher_obj_pos = rospy.Publisher('obj_pos', Point, queue_size = 10)

    def rois_callback(self, rois_msg):
        for roi in rois_msg.rois:
            mid_x=roi.x_offset +roi.width*0.5
            mid_y=roi.y_offset +roi.height *0.5
            mid_x=int(mid_x)
            mid_y=int(mid_y)
            depth_image = self.bridge.imgmsg_to_cv2(self.depth_iamge)
            msg = Point()
            if depth_image[mid_y][mid_x] != 0:
                camera_coordinate = rs.rs2_deproject_pixel_to_point(self.intrinsics, [mid_x,mid_y], depth_image[mid_y][mid_x]/10)
                msg.x, msg.y, msg.z = camera_coordinate[0], camera_coordinate[1], camera_coordinate[2]
                rospy.loginfo(f"发送: x={msg.x}, y={msg.y}, z={msg.z}")
                self.publisher_obj_pos.publish(msg)    

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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = RoisToPoint()
    node.run()
