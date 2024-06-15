#include "ros/ros.h"
#include "std_msgs/String.h"
#include "unitree_legged_msgs/HighCmd.h"

#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

class ListenerNode {
public:
    ListenerNode(ros::NodeHandle& nh) : nh_(nh) {
        sub_ = nh_.subscribe("Unitree_Highcmd", 1000, &ListenerNode::chatterCallback, this);
    }

    void chatterCallback(const unitree_legged_msgs::HighCmd::ConstPtr& highcmd) {
        if (highcmd->mode == 1){
            sport_client.StandUp();
        }else{
            sport_client.StandDown();
        }
        sport_client.Move(0.3, 0, 0.3);
        ROS_INFO("Mode: %u", highcmd->mode);
    }

    void spin() {
        ros::spin();
    }

private:
    unitree::robot::go2::SportClient sport_client;  
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv) {
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;

    ListenerNode listener(nh);

    listener.spin();

    return 0;
}
