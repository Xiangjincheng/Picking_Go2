import rospy

class Yolov5Pred:
    def __init__(self):
        rospy.init_node('yolov5_pred', anonymous=True)
        rospy.loginfo("节点:yolov5_pred, 已启动!")
        


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Yolov5Pred()
    node.run()
