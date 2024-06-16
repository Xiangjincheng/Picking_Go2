import rospy

class JakaController:
    def __init__(self):
        rospy.init_node('jaka_controller', anonymous=True)
        rospy.loginfo("节点:jaka_controller, 已启动!")
        


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = JakaController()
    node.run()
