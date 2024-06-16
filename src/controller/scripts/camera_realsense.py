import rospy

class RealSense:
    def __init__(self):
        rospy.init_node('realsense', anonymous=True)
        rospy.loginfo("节点:realsense, 已启动!")
        


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = RealSense()
    node.run()
